#!/usr/bin/env python3
"""
Acceleration Model Fitting from Existing Test Data
====================================================

Loads all force characterization and wall approach test runs, computes
actual acceleration from encoder velocity, and fits a model:

    a_predicted = f(v, voltage, brake_intensity)

The model captures the physics of a DC motor with H-bridge control:
    - Forward driving: torque ∝ (V_applied - back_EMF)
    - Coast friction: Coulomb + viscous drag
    - Active braking: short-circuit braking ∝ brake_intensity × |v|

Usage:
    python3 scripts/fit_acceleration_model.py
    python3 scripts/fit_acceleration_model.py --plot   # show plots interactively
"""

import pickle
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score, mean_absolute_error
from pathlib import Path
import matplotlib
import matplotlib.pyplot as plt
import argparse
import sys
import yaml


def load_force_char_tests(base_dir: Path) -> list[dict]:
    """Load all force characterization test runs with valid data."""
    all_tests = []
    for pkl in sorted(base_dir.glob("force_char_data_*.pkl")):
        try:
            with open(pkl, 'rb') as f:
                data = pickle.load(f)
            if isinstance(data, dict) and 'results' in data:
                for r in data['results']:
                    if r.get('valid', True) and len(r.get('data', [])) >= 10:
                        r['source'] = str(pkl.name)
                        all_tests.append(r)
        except Exception:
            pass
    for subdir in sorted(base_dir.glob("run_*")):
        for pkl in sorted(subdir.glob("force_char_data_*.pkl")):
            try:
                with open(pkl, 'rb') as f:
                    data = pickle.load(f)
                if isinstance(data, dict) and 'results' in data:
                    for r in data['results']:
                        if r.get('valid', True) and len(r.get('data', [])) >= 10:
                            r['source'] = str(pkl.relative_to(base_dir))
                            all_tests.append(r)
            except Exception:
                pass
    return all_tests


def load_wall_approach_runs(base_dir: Path) -> list[dict]:
    """Load all wall approach runs."""
    all_runs = []
    for subdir in sorted(base_dir.glob("wall_approach_*")):
        pkl = subdir / "wall_approach_data.pkl"
        if pkl.exists():
            try:
                with open(pkl, 'rb') as f:
                    data = pickle.load(f)
                if len(data.get('data', [])) >= 5:
                    data['source'] = str(subdir.name)
                    all_runs.append(data)
            except Exception:
                pass
    return all_runs


def build_segments_from_force_char(tests: list[dict],
                                   window: int = 5,
                                   stride: int = 2) -> list[dict]:
    """Build acceleration segments from force characterization tests.

    Uses overlapping windows with linear fit of v(t) to get cleaner
    acceleration estimates than point-wise finite differences.

    Returns list of dicts with: v, voltage, brake_intensity, accel, mode.
    """
    segments = []

    for test in tests:
        d = test['data']
        t = np.array([x['time'] for x in d])
        v_l = np.array([x['left_speed'] for x in d])
        v_r = np.array([x['right_speed'] for x in d])
        v_avg = (v_l + v_r) / 2
        cmd_avg = np.array([(x['left_motor_command'] + x['right_motor_command']) / 2 for x in d])
        vbat = np.array([x['battery_voltage'] for x in d])
        voltage = cmd_avg * vbat
        mode = test['mode']
        param = test['param']

        peak_idx = np.argmax(np.abs(v_avg))

        # Brake intensity: only applies during decel phase of brake tests
        brake_intensity = np.zeros(len(t))
        if mode in ('pure_brake', 'prop_brake'):
            brake_intensity[peak_idx:] = param if mode == 'prop_brake' else 1.0

        for start in range(0, len(t) - window, stride):
            end = start + window
            t_win = t[start:end]
            v_win = v_avg[start:end]
            volt_win = voltage[start:end]
            brake_win = brake_intensity[start:end]

            if np.max(np.abs(v_win)) < 0.03:
                continue
            dt_range = t_win[-1] - t_win[0]
            if dt_range < 0.1:
                continue

            # Linear fit v(t) for acceleration
            p = np.polyfit(t_win, v_win, 1)
            accel = p[0]

            segments.append({
                'v': float(np.mean(v_win)),
                'voltage': float(np.mean(volt_win)),
                'brake_intensity': float(np.mean(brake_win)),
                'accel': float(accel),
                'mode': mode,
            })

    return segments


def build_segments_from_wall_approach(runs: list[dict],
                                      window: int = 5,
                                      stride: int = 2) -> list[dict]:
    """Build acceleration segments from wall approach runs."""
    segments = []

    for run in runs:
        d = run['data']
        t = np.array([x['time'] for x in d])
        v_avg = np.array([(x['left_speed'] + x['right_speed']) / 2 for x in d])
        cmd_avg = np.array([(x['left_motor_command'] + x['right_motor_command']) / 2 for x in d])
        vbat = np.array([x['battery_voltage'] for x in d])
        voltage = cmd_avg * vbat

        # Determine brake intensity from motor mode if available
        has_mode = 'left_motor_mode' in d[0]
        brake_intensity = np.zeros(len(t))
        if has_mode:
            for i, pt in enumerate(d):
                if pt['left_motor_mode'] == 2:  # brake mode
                    # In brake mode, motor_command holds brake intensity
                    brake_intensity[i] = abs(pt['left_motor_command'])

        for start in range(0, len(t) - window, stride):
            end = start + window
            t_win = t[start:end]
            v_win = v_avg[start:end]

            if np.max(np.abs(v_win)) < 0.03:
                continue
            dt_range = t_win[-1] - t_win[0]
            if dt_range < 0.1:
                continue

            p = np.polyfit(t_win, v_win, 1)
            accel = p[0]

            segments.append({
                'v': float(np.mean(v_win)),
                'voltage': float(np.mean(voltage[start:end])),
                'brake_intensity': float(np.mean(brake_intensity[start:end])),
                'accel': float(accel),
                'mode': 'wall_approach',
            })

    return segments


def build_features(v: np.ndarray, voltage: np.ndarray,
                   brake_intensity: np.ndarray) -> np.ndarray:
    """Build feature matrix for the linear model.

    Features:
        1. voltage (= motor_command × battery_voltage) → driving force
        2. v (velocity) → back-EMF + viscous friction
        3. sign(v) → Coulomb friction
        4. brake_intensity × |v| → regenerative braking force

    Physics:
        a = k_torque × voltage          (motor torque)
          - k_backemf × v               (back-EMF opposes motion)
          - f_coulomb × sign(v)         (constant friction)
          - k_brake × brake × |v|       (regenerative braking ∝ back-EMF)
    """
    sign_v = np.sign(v)
    sign_v[sign_v == 0] = 1.0
    brake_force = brake_intensity * np.abs(v)

    return np.column_stack([voltage, v, sign_v, brake_force])


def main():
    parser = argparse.ArgumentParser(description='Fit acceleration model from test data')
    parser.add_argument('--plot', action='store_true', help='Show plots interactively')
    parser.add_argument('--no-save', action='store_true', help='Do not save plots/model')
    args = parser.parse_args()

    if not args.plot:
        matplotlib.use('Agg')

    base_dir = Path(__file__).parent.parent / "test_outputs"

    # ================================================================
    # Load data
    # ================================================================
    print("Loading force characterization data...")
    force_tests = load_force_char_tests(base_dir)
    print(f"  {len(force_tests)} valid tests")

    print("Loading wall approach data...")
    wall_runs = load_wall_approach_runs(base_dir)
    print(f"  {len(wall_runs)} valid runs")

    # ================================================================
    # Build segments (windowed acceleration estimates)
    # ================================================================
    print("\nBuilding segments...")
    seg_force = build_segments_from_force_char(force_tests)
    seg_wall = build_segments_from_wall_approach(wall_runs)
    segments = seg_force + seg_wall
    print(f"  Force char: {len(seg_force)} segments")
    print(f"  Wall approach: {len(seg_wall)} segments")
    print(f"  Total: {len(segments)} segments")

    # Extract arrays
    v = np.array([s['v'] for s in segments])
    voltage = np.array([s['voltage'] for s in segments])
    brake = np.array([s['brake_intensity'] for s in segments])
    accel = np.array([s['accel'] for s in segments])
    mode = np.array([s['mode'] for s in segments])

    # Filter
    moving = (np.abs(v) > 0.02) | (np.abs(voltage) > 0.1)
    reasonable = np.abs(accel) < 15.0
    mask = moving & reasonable
    v, voltage, brake, accel, mode = v[mask], voltage[mask], brake[mask], accel[mask], mode[mask]
    print(f"  After filtering: {len(v)} segments")

    # ================================================================
    # Fit model
    # ================================================================
    X = build_features(v, voltage, brake)

    np.random.seed(42)
    n = len(v)
    idx = np.random.permutation(n)
    split = int(0.8 * n)
    train_idx, test_idx = idx[:split], idx[split:]

    X_train, X_test = X[train_idx], X[test_idx]
    y_train, y_test = accel[train_idx], accel[test_idx]

    model = LinearRegression()
    model.fit(X_train, y_train)

    y_pred_train = model.predict(X_train)
    y_pred_test = model.predict(X_test)

    r2_train = r2_score(y_train, y_pred_train)
    r2_test = r2_score(y_test, y_pred_test)
    mae_train = mean_absolute_error(y_train, y_pred_train)
    mae_test = mean_absolute_error(y_test, y_pred_test)

    feature_names = ['voltage', 'velocity', 'sign(v)', 'brake*|v|']

    print(f"\n{'='*60}")
    print(f"Acceleration Model Results")
    print(f"{'='*60}")
    print(f"  Train: R²={r2_train:.4f}, MAE={mae_train:.3f} m/s²  ({len(y_train)} samples)")
    print(f"  Test:  R²={r2_test:.4f}, MAE={mae_test:.3f} m/s²  ({len(y_test)} samples)")
    print(f"\n  Coefficients:")
    for name, coef in zip(feature_names, model.coef_):
        print(f"    {name:15s}: {coef:+.4f}")
    print(f"    {'intercept':15s}: {model.intercept_:+.4f}")

    # Per-mode breakdown
    print(f"\n  Per-mode test accuracy:")
    for m in sorted(set(mode[test_idx])):
        mask_m = mode[test_idx] == m
        if mask_m.sum() < 5:
            continue
        r2_m = r2_score(y_test[mask_m], y_pred_test[mask_m])
        mae_m = mean_absolute_error(y_test[mask_m], y_pred_test[mask_m])
        print(f"    {m:20s}: R²={r2_m:.3f}, MAE={mae_m:.3f} ({mask_m.sum()} samples)")

    # Physical sanity checks
    c_intercept = model.intercept_
    c_voltage = model.coef_[0]
    c_velocity = model.coef_[1]
    c_sign = model.coef_[2]
    c_brake = model.coef_[3]

    print(f"\n  Physical sanity checks:")
    for v_val, volt_val, brake_val, label in [
        (0.5, 0.0, 0.0, "coast at 0.5 m/s"),
        (1.0, 0.0, 0.0, "coast at 1.0 m/s"),
        (0.5, 2.0, 0.0, "2V drive at 0.5 m/s"),
        (0.5, 5.0, 0.0, "5V drive at 0.5 m/s"),
        (1.0, 5.0, 0.0, "5V drive at 1.0 m/s"),
        (0.0, 3.0, 0.0, "3V from standstill"),
        (0.5, 0.0, 0.5, "50% brake at 0.5 m/s"),
        (0.5, 0.0, 1.0, "full brake at 0.5 m/s"),
        (1.0, 0.0, 1.0, "full brake at 1.0 m/s"),
    ]:
        sign_v = 1.0 if v_val >= 0 else -1.0
        a = c_intercept + c_voltage * volt_val + c_velocity * v_val + c_sign * sign_v + c_brake * brake_val * abs(v_val)
        print(f"    {label:30s}: a = {a:+.2f} m/s²")

    # Compare with previously characterized coast model (c0=0.951, c1=0.794)
    print(f"\n  Comparison with characterized coast model (decel = 0.951 + 0.794*v):")
    for v_val in [0.5, 0.7, 1.0, 1.5]:
        char_decel = 0.951 + 0.794 * v_val  # positive deceleration
        model_accel = c_intercept + c_velocity * v_val + c_sign * 1.0
        print(f"    v={v_val}: characterized={-char_decel:+.2f}, model={model_accel:+.2f} m/s²")

    # ================================================================
    # C++ code generation
    # ================================================================
    print(f"\n{'='*60}")
    print("C++ implementation")
    print(f"{'='*60}")
    print(f"""
// Acceleration prediction model
// Fitted from {len(v)} segments across force char + wall approach data
// Test R² = {r2_test:.4f}, MAE = {mae_test:.3f} m/s²
// Surface: hardwood floor
//
// Input: velocity (m/s), motor_command [-1,1], battery_voltage (V), brake_intensity [0,1]
// Output: predicted acceleration (m/s²)
//
// For go() commands: voltage = motor_command * battery_voltage, brake = 0
// For brake() commands: voltage = 0, brake = brake_intensity
// For coast (go(0)): voltage = 0, brake = 0
//
float predict_acceleration(float v, float motor_command, float battery_voltage,
                           float brake_intensity = 0.0f) {{
    float voltage = motor_command * battery_voltage;
    float sign_v = (v >= 0.0f) ? 1.0f : -1.0f;
    float brake_force = brake_intensity * fabsf(v);
    return {c_intercept:+.4f}f
         + {c_voltage:+.4f}f * voltage
         + {c_velocity:+.4f}f * v
         + {c_sign:+.4f}f * sign_v
         + {c_brake:+.4f}f * brake_force;
}}

// Inverse: what motor_command (go() PWM) achieves a desired acceleration?
// Only valid when not using brake() — i.e., controlling via go() only.
float command_for_acceleration(float desired_accel, float v, float battery_voltage) {{
    float sign_v = (v >= 0.0f) ? 1.0f : -1.0f;
    float voltage_needed = (desired_accel - ({c_intercept:+.4f}f)
                          - ({c_velocity:+.4f}f) * v
                          - ({c_sign:+.4f}f) * sign_v)
                         / ({c_voltage:+.4f}f);
    return voltage_needed / battery_voltage;
}}

// Inverse: what brake_intensity achieves a desired (negative) acceleration?
// Only valid when motor_command = 0, v > 0.
float brake_for_acceleration(float desired_accel, float v) {{
    float sign_v = (v >= 0.0f) ? 1.0f : -1.0f;
    float coast_accel = {c_intercept:+.4f}f + {c_velocity:+.4f}f * v + {c_sign:+.4f}f * sign_v;
    float extra_decel_needed = coast_accel - desired_accel;  // positive if we need more decel
    float brake_intensity = extra_decel_needed / ({-c_brake:+.4f}f * fabsf(v));
    return fmaxf(0.0f, fminf(1.0f, brake_intensity));
}}
""")

    # ================================================================
    # Save model coefficients
    # ================================================================
    if not args.no_save:
        model_dict = {
            'description': 'Linear acceleration model: a = c0 + c1*voltage + c2*v + c3*sign(v) + c4*brake*|v|',
            'surface': 'hardwood floor',
            'date': '2026-02-08',
            'n_samples': int(len(v)),
            'n_tests': len(force_tests) + len(wall_runs),
            'test_r2': float(r2_test),
            'test_mae_m_s2': float(mae_test),
            'coefficients': {
                'intercept': float(c_intercept),
                'voltage': float(c_voltage),
                'velocity': float(c_velocity),
                'sign_v': float(c_sign),
                'brake_x_abs_v': float(c_brake),
            },
            'feature_descriptions': {
                'voltage': 'motor_command * battery_voltage (V)',
                'velocity': 'current velocity (m/s)',
                'sign_v': 'sign of velocity (+1 or -1)',
                'brake_x_abs_v': 'brake_intensity * |velocity|',
            },
        }
        model_path = base_dir / "acceleration_model.yaml"
        with open(model_path, 'w') as f:
            yaml.dump(model_dict, f, default_flow_style=False, sort_keys=False)
        print(f"\nSaved model to {model_path}")

    # ================================================================
    # Plots
    # ================================================================
    print("\nGenerating plots...")
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Acceleration Model — Fit Quality\n'
                 f'R²={r2_test:.3f}, MAE={mae_test:.3f} m/s² '
                 f'({len(v)} segments from {len(force_tests)} force + {len(wall_runs)} wall tests)',
                 fontsize=13)

    # 1. Predicted vs actual
    ax = axes[0, 0]
    ax.scatter(y_test, y_pred_test, alpha=0.3, s=10, c='steelblue')
    lims = [min(y_test.min(), y_pred_test.min()) - 0.5,
            max(y_test.max(), y_pred_test.max()) + 0.5]
    ax.plot(lims, lims, 'r--', lw=1, label='perfect')
    ax.set_xlabel('Actual acceleration (m/s²)')
    ax.set_ylabel('Predicted acceleration (m/s²)')
    ax.set_title('Predicted vs Actual')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 2. Residuals vs velocity
    ax = axes[0, 1]
    residuals = y_test - y_pred_test
    ax.scatter(v[test_idx], residuals, alpha=0.3, s=10, c='coral')
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('Velocity (m/s)')
    ax.set_ylabel('Residual (m/s²)')
    ax.set_title('Residuals vs Velocity')
    ax.grid(True, alpha=0.3)

    # 3. Residuals vs voltage
    ax = axes[0, 2]
    ax.scatter(voltage[test_idx], residuals, alpha=0.3, s=10, c='forestgreen')
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('Applied Voltage (V)')
    ax.set_ylabel('Residual (m/s²)')
    ax.set_title('Residuals vs Voltage')
    ax.grid(True, alpha=0.3)

    # 4. Per-mode box plot
    ax = axes[1, 0]
    unique_modes = sorted(set(mode[test_idx]))
    mode_residuals = []
    mode_labels = []
    for m in unique_modes:
        mask_m = mode[test_idx] == m
        if mask_m.sum() >= 3:
            mode_residuals.append(residuals[mask_m])
            mode_labels.append(m)
    if mode_residuals:
        bp = ax.boxplot(mode_residuals, tick_labels=mode_labels, patch_artist=True)
        colors = plt.cm.Set3(np.linspace(0, 1, len(mode_labels)))
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
    ax.axhline(0, color='k', lw=0.5)
    ax.set_ylabel('Residual (m/s²)')
    ax.set_title('Residuals by Mode')
    ax.tick_params(axis='x', rotation=45)
    ax.grid(True, alpha=0.3, axis='y')

    # 5. Model prediction surface: accel vs velocity for various voltages
    ax = axes[1, 1]
    v_range = np.linspace(0, 2.0, 100)
    for volt_val, color, label in [
        (0.0, 'gray', 'coast (0V)'),
        (2.0, 'skyblue', '2V'),
        (4.0, 'dodgerblue', '4V'),
        (6.0, 'blue', '6V'),
        (8.0, 'darkblue', '8V'),
    ]:
        X_pred = build_features(v_range, np.full_like(v_range, volt_val),
                                np.zeros_like(v_range))
        a_pred = model.predict(X_pred)
        ax.plot(v_range, a_pred, color=color, lw=2, label=label)

    # Overlay brake curves
    for brake_val, ls, label in [
        (0.5, '--', '50% brake'),
        (1.0, ':', 'full brake'),
    ]:
        X_pred = build_features(v_range, np.zeros_like(v_range),
                                np.full_like(v_range, brake_val))
        a_pred = model.predict(X_pred)
        ax.plot(v_range, a_pred, color='red', ls=ls, lw=2, label=label)

    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('Velocity (m/s)')
    ax.set_ylabel('Predicted acceleration (m/s²)')
    ax.set_title('Model Prediction Surface')
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    # 6. Acceleration histogram colored by mode
    ax = axes[1, 2]
    for m in sorted(set(mode)):
        mask_m = mode == m
        if mask_m.sum() < 10:
            continue
        ax.hist(accel[mask_m], bins=50, alpha=0.5, label=m, density=True)
    ax.set_xlabel('Acceleration (m/s²)')
    ax.set_ylabel('Density')
    ax.set_title('Acceleration Distribution by Mode')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if not args.no_save:
        plot_path = base_dir / "acceleration_model_fit.png"
        plt.savefig(plot_path, dpi=150)
        print(f"Saved plot to {plot_path}")

    if args.plot:
        plt.show()

    # ================================================================
    # Data coverage summary
    # ================================================================
    print(f"\n{'='*60}")
    print("Data coverage")
    print(f"{'='*60}")
    print(f"  Velocity range:  [{v.min():.2f}, {v.max():.2f}] m/s")
    print(f"  Voltage range:   [{voltage.min():.2f}, {voltage.max():.2f}] V")
    print(f"  Brake range:     [{brake.min():.2f}, {brake.max():.2f}]")
    print(f"  Accel range:     [{accel.min():.2f}, {accel.max():.2f}] m/s²")
    print(f"\n  Samples per mode:")
    for m in sorted(set(mode)):
        print(f"    {m:20s}: {(mode == m).sum():6d}")

    # ================================================================
    # Limitations & next steps
    # ================================================================
    print(f"\n{'='*60}")
    print("Limitations of this model")
    print(f"{'='*60}")
    print("  - Trained on hardwood floor only — friction differs on other surfaces")
    print("  - 10Hz data limits acceleration resolution (~0.5 m/s² noise floor)")
    print("  - No BNO accelerometer data in stored pkl files (would improve resolution)")
    print("  - Brake characterization limited to force_char tests (not RC driving)")
    print("  - Linear model — may miss nonlinearities at very low/high speeds")
    print("  - Left/right wheels averaged — asymmetry not captured")
    print()
    print("  Next steps to improve:")
    print("  1. Log BNO accelerometer data in pkl files for better accel estimates")
    print("  2. Collect driving data with RC (HandMode) — covers realistic transitions")
    print("  3. Try per-wheel models if tracking error is asymmetric")
    print("  4. Test on different surfaces to see friction variation")


if __name__ == '__main__':
    main()
