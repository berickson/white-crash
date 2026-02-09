#!/usr/bin/env python3
"""
Acceleration Characterization Test & Analysis Pipeline
=======================================================

Collects (velocity, pwm, acceleration, battery_voltage) data by applying
open-loop PWM steps at various starting velocities. Fits the inverse model:
  pwm = f(v, a_desired)

which is needed for the feedforward term in the revised control plan.

Usage:
    # Run all 10 ground tests (manual repositioning between each)
    python3 run_accel_characterization.py --run

    # Run single test
    python3 run_accel_characterization.py --single 0.0 0.35

    # Analyze existing data file
    python3 run_accel_characterization.py --analyze test_outputs/accel_char_.../accel_char_data_*.pkl

Requirements:
    - Robot in hand mode, on ground
    - micro-ROS agent running
    - Position robot ~1.5m from wall for each test (pointed toward wall)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from white_crash_msgs.msg import Update
from rcl_interfaces.msg import Log
import argparse
import time
import pickle
import numpy as np
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from datetime import datetime
import sys
import threading
import yaml
import os
from pathlib import Path


# Global variable to hold the output directory for the current run
_run_output_dir = None


def get_output_dir(timestamp=None, create=True):
    """Get the output directory for this run, creating if needed."""
    global _run_output_dir

    if _run_output_dir is None:
        if timestamp is None:
            timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

        script_dir = Path(__file__).parent.parent
        _run_output_dir = script_dir / "test_outputs" / f"accel_char_{timestamp}"

        if create:
            _run_output_dir.mkdir(parents=True, exist_ok=True)
            print(f"Output directory: {_run_output_dir}")

    return _run_output_dir


def output_path(filename):
    """Get full path for an output file in the current run's directory."""
    return str(get_output_dir() / filename)


class AccelTestClient(Node):
    """Client for running acceleration characterization tests"""

    def __init__(self):
        super().__init__('accel_test_client')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.command_pub = self.create_publisher(
            String,
            '/white_crash/command',
            qos_profile
        )

        self.response_sub = self.create_subscription(
            String,
            '/white_crash/command_response',
            self.response_callback,
            qos_profile
        )

        self.update_sub = self.create_subscription(
            Update,
            '/white_crash/update',
            self.update_callback,
            qos_profile
        )

        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout_best_effort',
            self.rosout_callback,
            qos_profile
        )

        self.current_test_data = []
        self.collecting = False
        self.test_start_time = None

        self.last_response = None
        self.response_received = threading.Event()

    def rosout_callback(self, msg):
        if 'white_crash' in msg.name or msg.name == '':
            print(f"  [ROBOT] {msg.msg}")

    def response_callback(self, msg):
        self.last_response = msg.data
        self.response_received.set()

    def update_callback(self, msg):
        if self.collecting:
            current_time = time.time() - self.test_start_time
            self.current_test_data.append({
                'time': current_time,
                'left_speed': msg.left_speed,
                'right_speed': msg.right_speed,
                'left_motor_command': msg.left_motor_command,
                'right_motor_command': msg.right_motor_command,
                'left_motor_mode': msg.left_motor_mode,
                'right_motor_mode': msg.right_motor_mode,
                'battery_voltage': msg.battery_voltage,
                'bno_acceleration_x': msg.bno_acceleration_x,
                'bno_acceleration_y': msg.bno_acceleration_y,
                'bno_gyro_z': msg.bno_gyro_z,
            })

    def send_command(self, command_text, timeout=3.0, retries=2):
        for attempt in range(retries + 1):
            msg = String()
            msg.data = command_text
            self.command_pub.publish(msg)

            self.response_received.clear()
            if self.response_received.wait(timeout=timeout):
                return self.last_response
            elif attempt < retries:
                self.get_logger().warning(f"Command timeout, retry {attempt + 1}/{retries}")
                time.sleep(0.5)
        return None

    def wait_for_robot_ready(self, timeout=10.0):
        self.get_logger().info("Checking robot connectivity...")
        start = time.time()
        initial_count = len(self.current_test_data) if self.collecting else 0

        was_collecting = self.collecting
        self.collecting = True
        self.test_start_time = time.time()

        while time.time() - start < timeout:
            time.sleep(0.5)
            if len(self.current_test_data) > initial_count + 2:
                self.collecting = was_collecting
                self.current_test_data = []
                self.get_logger().info("Robot is responsive")
                return True

        self.collecting = was_collecting
        self.current_test_data = []
        self.get_logger().error("Robot not responding!")
        return False

    def run_single_test(self, start_velocity, pwm_level, rest_time=2.0):
        """Run a single acceleration characterization test."""
        self.get_logger().info(
            f"Running accel test: start_v={start_velocity} m/s, pwm={pwm_level}")

        # Configure the test
        response = self.send_command(f"accel-test {start_velocity} {pwm_level}")
        if response is None or "ERROR" in response:
            self.get_logger().error(f"Failed to configure test: {response}")
            return None
        self.get_logger().info(f"  Config response: {response}")

        # Clear data and start collecting
        self.current_test_data = []
        self.collecting = True
        self.test_start_time = time.time()

        # Trigger accel-char mode
        response = self.send_command("set-event accel-char")
        if response is None:
            self.get_logger().error("FAILED to trigger accel-char - no response!")
            time.sleep(0.5)
            response = self.send_command("set-event accel-char")
            if response is None:
                self.get_logger().error("Second attempt also failed!")
        else:
            self.get_logger().info(f"  Event response: {response}")

        # Wait for test to complete
        max_wait = 12.0  # Enough for accel + settle + test
        start = time.time()
        got_velocity = False
        motors_were_active = False

        while time.time() - start < max_wait:
            time.sleep(0.1)

            if len(self.current_test_data) > 5:
                recent = self.current_test_data[-5:]
                avg_speed = sum(
                    d['left_speed'] + d['right_speed'] for d in recent
                ) / (2 * len(recent))
                avg_cmd = sum(
                    abs(d['left_motor_command']) + abs(d['right_motor_command'])
                    for d in recent
                ) / (2 * len(recent))

                if avg_cmd > 0.05:
                    motors_were_active = True

                if avg_speed > 0.1:
                    got_velocity = True

                # Test complete: had velocity, now stopped, motors off
                if got_velocity and avg_speed < 0.05 and motors_were_active:
                    # Check if motor command also went to zero (test ended)
                    if avg_cmd < 0.05:
                        time.sleep(0.2)
                        break

        # Stop collecting
        self.collecting = False

        # Return to hand mode
        self.send_command("set-event hand")
        time.sleep(0.5)
        self.send_command("set-event hand")

        # Rest before next test
        time.sleep(rest_time)

        # Validate
        peak_velocity = 0.0
        if self.current_test_data:
            speeds = [
                (d['left_speed'] + d['right_speed']) / 2
                for d in self.current_test_data
            ]
            peak_velocity = max(speeds) if speeds else 0.0

        result = {
            'start_velocity': start_velocity,
            'pwm_level': pwm_level,
            'data': self.current_test_data.copy(),
            'timestamp': datetime.now().isoformat(),
            'peak_velocity': peak_velocity,
            'valid': len(self.current_test_data) > 10,
        }

        self.get_logger().info(
            f"Test complete: {len(result['data'])} samples, "
            f"peak={peak_velocity:.2f} m/s, valid={result['valid']}")
        return result


def build_test_sequence():
    """Build the 10-test ground sequence."""
    tests = [
        # (start_velocity, pwm_level)
        # From standstill — sweep PWM to capture full accel curves
        (0.0, 0.15),   # 1: Gentle accel
        (0.0, 0.25),   # 2: Low-mid
        (0.0, 0.35),   # 3: Mid
        (0.0, 0.50),   # 4: Near slip boundary
        # From speed — step changes to capture mid-velocity behavior
        (0.5, 0.70),   # 5: Step-up from slow
        (0.5, 0.30),   # 6: Partial power from slow
        (1.0, 0.80),   # 7: Full power from cruising
        (1.0, 0.30),   # 8: Step-down
        (1.5, 0.40),   # 9: Step-down from fast
        (0.7, 0.50),   # 10: Mid-range fill
    ]
    return tests


def run_test_suite():
    """Run the full 10-test ground suite with manual repositioning."""
    print(f"\n{'='*70}")
    print("Acceleration Characterization Test Suite - GROUND")
    print(f"{'='*70}\n")

    rclpy.init()
    client = AccelTestClient()

    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()

    tests = build_test_sequence()
    print(f"Total tests to run: {len(tests)}")
    print("\nFor each test, position robot ~1.5m from wall, pointed toward it.\n")

    all_results = []
    failed_tests = []

    try:
        print("Connecting to robot...")
        time.sleep(2.0)

        for i, (start_v, pwm) in enumerate(tests):
            print(f"\n[{i+1}/{len(tests)}] Accel test: start_v={start_v} m/s, pwm={pwm}")
            input("  >>> Position robot ~1.5m from wall, then press ENTER...")

            if not client.wait_for_robot_ready(timeout=5.0):
                print("  ⚠ Robot not responding, skipping test")
                failed_tests.append((start_v, pwm))
                continue

            result = client.run_single_test(start_v, pwm)

            if result and result.get('valid', False):
                all_results.append(result)
                # Save incrementally
                temp_file = output_path("accel_char_progress.pkl")
                with open(temp_file, 'wb') as f:
                    pickle.dump({'results': all_results}, f)
                print(f"  ✓ Valid test, saved progress ({len(all_results)} good tests)")
            else:
                peak = result.get('peak_velocity', 0) if result else 0
                print(f"  ✗ Invalid test (peak={peak:.2f}), will retry later")
                failed_tests.append((start_v, pwm))

        # Retry failed tests
        if failed_tests:
            print(f"\n{'='*70}")
            print(f"Retrying {len(failed_tests)} failed tests...")
            print(f"{'='*70}")

            still_failed = []
            for start_v, pwm in failed_tests:
                print(f"  Retrying: start_v={start_v}, pwm={pwm}")
                input("  >>> Position robot ~1.5m from wall, then press ENTER...")

                result = client.run_single_test(start_v, pwm, rest_time=3.0)

                if result and result.get('valid', False):
                    all_results.append(result)
                    temp_file = output_path("accel_char_progress.pkl")
                    with open(temp_file, 'wb') as f:
                        pickle.dump({'results': all_results}, f)
                    print(f"    ✓ Retry succeeded!")
                else:
                    still_failed.append((start_v, pwm))
                    print(f"    ✗ Retry failed")

            failed_tests = still_failed

        print(f"\n{'='*70}")
        print(f"All tests complete! {len(all_results)} valid, {len(failed_tests)} failed")
        print(f"{'='*70}\n")

        # Save final results
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        data_file = output_path(f"accel_char_data_{timestamp}.pkl")
        with open(data_file, 'wb') as f:
            pickle.dump({'results': all_results, 'timestamp': timestamp}, f)
        print(f"Data saved to: {data_file}")

        return data_file

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        data_file = output_path(f"accel_char_data_{timestamp}_interrupted.pkl")
        with open(data_file, 'wb') as f:
            pickle.dump({'results': all_results, 'timestamp': timestamp}, f)
        print(f"Partial data saved to: {data_file}")
        return data_file

    finally:
        client.destroy_node()
        rclpy.shutdown()


def run_single_test(start_velocity, pwm_level):
    """Run a single test for debugging."""
    print(f"\n{'='*70}")
    print(f"Single Accel Test: start_v={start_velocity} m/s, pwm={pwm_level}")
    print(f"{'='*70}\n")

    rclpy.init()
    client = AccelTestClient()

    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()

    try:
        print("Connecting to robot...")
        time.sleep(2.0)

        result = client.run_single_test(start_velocity, pwm_level)

        if result and result['data']:
            print(f"\nTest complete: {len(result['data'])} samples")

            times = [d['time'] for d in result['data']]
            speeds = [(d['left_speed'] + d['right_speed']) / 2 for d in result['data']]
            cmds = [(d['left_motor_command'] + d['right_motor_command']) / 2
                    for d in result['data']]

            fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
            axes[0].plot(times, speeds, 'b-', linewidth=2)
            axes[0].set_ylabel('Velocity (m/s)')
            axes[0].set_title(f'Accel test: start_v={start_velocity}, pwm={pwm_level}')
            axes[0].grid(True, alpha=0.3)

            axes[1].plot(times, cmds, 'r-', linewidth=2)
            axes[1].set_ylabel('Motor Command')
            axes[1].set_xlabel('Time (s)')
            axes[1].grid(True, alpha=0.3)

            plt.tight_layout()
            plot_file = output_path(
                f'single_accel_test_v{start_velocity}_pwm{pwm_level}.png')
            plt.savefig(plot_file)
            plt.show()
            print(f"Plot saved to: {plot_file}")
        else:
            print("Test failed!")

    finally:
        client.destroy_node()
        rclpy.shutdown()


class AccelCharacterizationAnalyzer:
    """Analyzes collected acceleration characterization data.

    From each test run we extract (velocity, pwm, acceleration, battery_voltage)
    tuples. These are used to fit the inverse model: pwm = f(v, a).
    """

    def __init__(self, data_file):
        with open(data_file, 'rb') as f:
            self.data_dict = pickle.load(f)

        self.results = self.data_dict.get('results', [])
        print(f"Loaded data from {data_file}")
        print(f"Total tests: {len(self.results)}")

    def extract_accel_data(self, result, min_velocity=0.03):
        """Extract (v, pwm, accel, v_bat, imu_accel_x) tuples from a test.

        Uses Savitzky-Golay filter on velocity then differentiates.
        Returns arrays: v, pwm, accel, v_bat, imu_ax
        """
        data = result['data']
        if len(data) < 10:
            return None

        t = np.array([d['time'] for d in data])
        left_v = np.array([d['left_speed'] for d in data])
        right_v = np.array([d['right_speed'] for d in data])
        avg_v = (left_v + right_v) / 2.0
        left_cmd = np.array([d['left_motor_command'] for d in data])
        right_cmd = np.array([d['right_motor_command'] for d in data])
        avg_cmd = (left_cmd + right_cmd) / 2.0
        v_bat = np.array([d['battery_voltage'] for d in data])
        imu_ax = np.array([d['bno_acceleration_x'] for d in data])

        # Smooth velocity before differentiating
        if len(avg_v) > 15:
            window = min(11, len(avg_v) // 2 * 2 + 1)
            v_smooth = savgol_filter(avg_v, window, min(3, window - 1))
        else:
            v_smooth = avg_v

        # Compute acceleration via finite difference
        dt = np.diff(t)
        dv = np.diff(v_smooth)
        accel = dv / np.where(dt > 0, dt, 1e-6)

        # Use midpoints for v, pwm, v_bat, imu
        v_mid = (avg_v[:-1] + avg_v[1:]) / 2
        pwm_mid = (avg_cmd[:-1] + avg_cmd[1:]) / 2
        vbat_mid = (v_bat[:-1] + v_bat[1:]) / 2
        imu_mid = (imu_ax[:-1] + imu_ax[1:]) / 2

        # Filter: only use points with velocity > min and reasonable accel
        mask = (v_mid > min_velocity) & (np.abs(accel) < 30)

        return {
            'v': v_mid[mask],
            'pwm': pwm_mid[mask],
            'accel': accel[mask],
            'v_bat': vbat_mid[mask],
            'imu_ax': imu_mid[mask],
        }

    def collect_all_data(self):
        """Collect (v, pwm, accel, v_bat, imu_ax) from all tests."""
        all_v, all_pwm, all_accel, all_vbat, all_imu = [], [], [], [], []
        meta = []  # Track which test each point came from

        for i, result in enumerate(self.results):
            extracted = self.extract_accel_data(result)
            if extracted is None:
                continue

            n = len(extracted['v'])
            all_v.append(extracted['v'])
            all_pwm.append(extracted['pwm'])
            all_accel.append(extracted['accel'])
            all_vbat.append(extracted['v_bat'])
            all_imu.append(extracted['imu_ax'])
            meta.extend([i] * n)

        if not all_v:
            print("WARNING: No valid data extracted!")
            return None

        return {
            'v': np.concatenate(all_v),
            'pwm': np.concatenate(all_pwm),
            'accel': np.concatenate(all_accel),
            'v_bat': np.concatenate(all_vbat),
            'imu_ax': np.concatenate(all_imu),
            'test_idx': np.array(meta),
        }

    def detect_slip(self, data, threshold=3.0):
        """Flag samples where encoder accel and IMU accel disagree.

        Returns boolean mask where True = likely slipping.

        threshold: max allowed difference between encoder-derived and IMU
                   acceleration (m/s²) before flagging as slip.
        """
        # IMU x-axis is forward acceleration (positive = forward)
        diff = np.abs(data['accel'] - data['imu_ax'])
        slip_mask = diff > threshold
        n_slip = np.sum(slip_mask)
        if n_slip > 0:
            print(f"  Slip detection: {n_slip}/{len(slip_mask)} samples flagged "
                  f"({100*n_slip/len(slip_mask):.1f}%)")
        return slip_mask

    def fit_model(self, gate_slip=True, slip_threshold=3.0):
        """Fit the inverse model: voltage = f(v, a).

        We model motor voltage (not PWM) to factor out battery variation:
          V_motor = v0 + v1 * velocity + v2 * acceleration

        Then at runtime: pwm = V_motor / V_battery

        Returns fitted parameters and diagnostics.
        """
        data = self.collect_all_data()
        if data is None:
            return None

        # Gate out slip if requested
        if gate_slip:
            slip = self.detect_slip(data, threshold=slip_threshold)
            mask = ~slip
            print(f"  Using {np.sum(mask)}/{len(mask)} non-slip samples for fitting")
        else:
            mask = np.ones(len(data['v']), dtype=bool)

        v = data['v'][mask]
        accel = data['accel'][mask]
        pwm = data['pwm'][mask]
        v_bat = data['v_bat'][mask]

        # Convert PWM to voltage: V_motor = pwm * V_battery
        v_motor = pwm * v_bat

        # Fit: V_motor = c0 + c1 * v + c2 * a
        def model(X, c0, c1, c2):
            vel, acc = X
            return c0 + c1 * vel + c2 * acc

        try:
            popt, pcov = curve_fit(model, (v, accel), v_motor,
                                   p0=[0.3, 1.5, 0.5], maxfev=10000)
            perr = np.sqrt(np.diag(pcov))

            # Compute residuals
            v_predicted = model((v, accel), *popt)
            residuals = v_motor - v_predicted
            rmse = np.sqrt(np.mean(residuals**2))

            print(f"\n  Fitted model: V_motor = {popt[0]:.4f} + {popt[1]:.4f}*v + {popt[2]:.4f}*a")
            print(f"  Std errors:              ±{perr[0]:.4f}   ±{perr[1]:.4f}    ±{perr[2]:.4f}")
            print(f"  RMSE: {rmse:.4f} V")
            print(f"  Data points: {len(v)}")

            result = {
                'c0': float(popt[0]),
                'c1': float(popt[1]),
                'c2': float(popt[2]),
                'c0_err': float(perr[0]),
                'c1_err': float(perr[1]),
                'c2_err': float(perr[2]),
                'rmse_volts': float(rmse),
                'num_points': int(len(v)),
                'description': 'V_motor = c0 + c1*velocity + c2*acceleration; pwm = V_motor / V_battery',
            }

            return result, data, mask

        except Exception as e:
            print(f"  Curve fit failed: {e}")
            return None

    def generate_report(self, output_pdf):
        """Generate comprehensive PDF report."""
        print(f"\nGenerating report: {output_pdf}")

        with PdfPages(output_pdf) as pdf:
            # Title page
            fig, ax = plt.subplots(figsize=(11, 8.5))
            ax.text(0.5, 0.9, 'Acceleration Characterization Report',
                    ha='center', fontsize=20, fontweight='bold',
                    transform=ax.transAxes)
            ax.text(0.5, 0.7, f'Total tests: {len(self.results)}',
                    ha='center', fontsize=14, transform=ax.transAxes)
            ax.text(0.5, 0.6, f'Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}',
                    ha='center', fontsize=12, transform=ax.transAxes)
            ax.axis('off')
            pdf.savefig(fig)
            plt.close()

            # Per-test velocity and command plots
            fig, axes = plt.subplots(2, 1, figsize=(11, 8.5), sharex=True)
            fig.suptitle('All Tests: Velocity & Motor Command vs Time', fontsize=14)

            for result in self.results:
                data = result['data']
                if not data:
                    continue
                t = [d['time'] for d in data]
                v = [(d['left_speed'] + d['right_speed']) / 2 for d in data]
                cmd = [(d['left_motor_command'] + d['right_motor_command']) / 2
                       for d in data]

                label = f"v0={result['start_velocity']}, pwm={result['pwm_level']}"
                axes[0].plot(t, v, alpha=0.7, linewidth=1.5, label=label)
                axes[1].plot(t, cmd, alpha=0.7, linewidth=1.5)

            axes[0].set_ylabel('Velocity (m/s)')
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(fontsize=7, ncol=2)
            axes[1].set_ylabel('Motor Command (PWM)')
            axes[1].set_xlabel('Time (s)')
            axes[1].grid(True, alpha=0.3)
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close()

            # Fit model and plot diagnostics
            fit_result = self.fit_model()
            if fit_result is not None:
                params, data, mask = fit_result

                v = data['v'][mask]
                accel = data['accel'][mask]
                pwm = data['pwm'][mask]
                v_bat = data['v_bat'][mask]
                v_motor = pwm * v_bat
                test_idx = data['test_idx'][mask]

                # Plot: V_motor vs velocity, colored by acceleration
                fig, axes = plt.subplots(1, 2, figsize=(11, 5))
                sc = axes[0].scatter(v, v_motor, c=accel, cmap='coolwarm',
                                     s=10, alpha=0.5)
                plt.colorbar(sc, ax=axes[0], label='Acceleration (m/s²)')
                axes[0].set_xlabel('Velocity (m/s)')
                axes[0].set_ylabel('V_motor (V)')
                axes[0].set_title('Motor Voltage vs Velocity (color=accel)')
                axes[0].grid(True, alpha=0.3)

                # Plot: predicted vs actual
                v_pred = params['c0'] + params['c1'] * v + params['c2'] * accel
                axes[1].scatter(v_motor, v_pred, s=10, alpha=0.3)
                lims = [min(v_motor.min(), v_pred.min()),
                        max(v_motor.max(), v_pred.max())]
                axes[1].plot(lims, lims, 'r--', linewidth=1)
                axes[1].set_xlabel('Actual V_motor (V)')
                axes[1].set_ylabel('Predicted V_motor (V)')
                axes[1].set_title(f'Predicted vs Actual (RMSE={params["rmse_volts"]:.3f}V)')
                axes[1].grid(True, alpha=0.3)

                plt.tight_layout()
                pdf.savefig(fig)
                plt.close()

                # Slip detection plot
                slip = self.detect_slip(data)
                fig, ax = plt.subplots(figsize=(11, 5))
                diff = np.abs(data['accel'] - data['imu_ax'])
                ax.scatter(data['v'][~slip], diff[~slip], s=8, alpha=0.3,
                           label='Clean', color='blue')
                ax.scatter(data['v'][slip], diff[slip], s=12, alpha=0.5,
                           label='Slip flagged', color='red')
                ax.set_xlabel('Velocity (m/s)')
                ax.set_ylabel('|encoder_accel - imu_accel| (m/s²)')
                ax.set_title('Slip Detection: Encoder vs IMU Acceleration Disagreement')
                ax.legend()
                ax.grid(True, alpha=0.3)
                plt.tight_layout()
                pdf.savefig(fig)
                plt.close()

                # Residuals by test
                fig, ax = plt.subplots(figsize=(11, 5))
                residuals = v_motor - v_pred
                unique_tests = np.unique(test_idx)
                for ti in unique_tests:
                    m = test_idx == ti
                    r = self.results[int(ti)]
                    label = f"v0={r['start_velocity']}, pwm={r['pwm_level']}"
                    ax.scatter(v[m], residuals[m], s=10, alpha=0.5, label=label)
                ax.axhline(0, color='k', linewidth=0.5)
                ax.set_xlabel('Velocity (m/s)')
                ax.set_ylabel('Residual (V)')
                ax.set_title('Model Residuals by Test')
                ax.legend(fontsize=7, ncol=2)
                ax.grid(True, alpha=0.3)
                plt.tight_layout()
                pdf.savefig(fig)
                plt.close()

        print(f"Report saved to {output_pdf}")

    def export_model(self, output_yaml):
        """Fit model and export to YAML."""
        fit_result = self.fit_model()
        if fit_result is None:
            print("Cannot export model - fitting failed")
            return

        params, data, mask = fit_result

        model = {
            'metadata': {
                'description': 'Acceleration-to-voltage model for feedforward control',
                'equation': 'V_motor = c0 + c1*velocity + c2*acceleration',
                'usage': 'pwm = V_motor / V_battery',
                'timestamp': self.data_dict.get('timestamp', 'unknown'),
                'num_tests': len(self.results),
            },
            'model': {
                'c0': params['c0'],
                'c1': params['c1'],
                'c2': params['c2'],
                'c0_err': params['c0_err'],
                'c1_err': params['c1_err'],
                'c2_err': params['c2_err'],
                'rmse_volts': params['rmse_volts'],
                'num_points': params['num_points'],
            }
        }

        with open(output_yaml, 'w') as f:
            yaml.dump(model, f, default_flow_style=False)

        print(f"Model exported to {output_yaml}")


def analyze_data(data_file):
    """Analyze existing data file."""
    print(f"\n{'='*70}")
    print("Acceleration Characterization Analysis")
    print(f"{'='*70}\n")

    analyzer = AccelCharacterizationAnalyzer(data_file)

    timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

    report_pdf = output_path(f"accel_char_report_{timestamp}.pdf")
    analyzer.generate_report(report_pdf)

    model_yaml = output_path(f"accel_model_{timestamp}.yaml")
    analyzer.export_model(model_yaml)

    print(f"\n{'='*70}")
    print("Analysis complete!")
    print(f"  Report: {report_pdf}")
    print(f"  Model:  {model_yaml}")
    print(f"{'='*70}\n")


def main():
    parser = argparse.ArgumentParser(
        description='Acceleration characterization test and analysis',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--run', action='store_true',
                       help='Run full 10-test ground suite')
    group.add_argument('--single', nargs=2,
                       metavar=('START_VELOCITY', 'PWM_LEVEL'),
                       help='Run single test: --single 0.0 0.35')
    group.add_argument('--analyze', metavar='FILE',
                       help='Analyze existing data file')

    args = parser.parse_args()

    if args.analyze:
        analyze_data(args.analyze)
    elif args.single:
        start_v, pwm = args.single
        run_single_test(float(start_v), float(pwm))
    else:
        data_file = run_test_suite()
        if data_file:
            analyze_data(data_file)


if __name__ == '__main__':
    main()
