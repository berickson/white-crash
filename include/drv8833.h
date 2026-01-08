// controls one side of a drv8833 motor controller
//
// see https://www.ti.com/lit/ds/symlink/drv8833.pdf

#include "Arduino.h"
#include "utils.h"


class DRV8833 {
    public:
        void init(int pin_fwd , int pin_rev, int frequency = 30000, int resolution = 10) {
            this->pin_fwd = pin_fwd;
            this->pin_rev = pin_rev;
            this->frequency = frequency;
            this->resolution = resolution;
            pinMode(pin_fwd, OUTPUT);
            pinMode(pin_rev, OUTPUT);
            analogWriteFrequency(frequency);
            analogWriteResolution(resolution);
        }

        inline float get_setpoint() {
            return setpoint;
        }

        // rate is [-1.0,1.0], fast_decay will cause faster
        // deceleration
        // NOTE: fast_decay parameter is deprecated and should be removed
        // It doesn't make sense to use fast_decay with active throttle (they fight each other)
        // Current code (go to can mode) still uses it, so keeping for compatibility
        // All new code should pass fast_decay=false
        // TODO: Remove fast_decay parameter when client code is refactored
        void go(float rate, bool fast_decay = false) {
            setpoint = rate;
            pinMode(pin_fwd, OUTPUT);
            pinMode(pin_rev, OUTPUT);
            rate = clamp(rate, -1.0f, 1.0f);
            // Serial.printf("rate: %0.4f %d\n", rate, fast_decay);

            if (rate > 0.0) {
                forward(rate, fast_decay);
                return;
            }
            else if (rate < 0.0) {
                reverse(fabs(rate), fast_decay);
                return;
            }
            if (fast_decay) {
                brake(1.0f);
            } else {
                coast();
            }
        }

        // Proportional braking control
        // intensity ∈ [0, 1]
        //   0.0 = full coast (both pins LOW)
        //   1.0 = full brake (both pins HIGH) 
        //   0 < intensity < 1 = PWM brake duty cycle
        // Use brake(1.0) for position holding when stopped
        void brake(float intensity) {
            intensity = clamp(intensity, 0.0f, 1.0f);
            
            if (intensity <= 0.0) {
                coast();
                return;
            }
            
            // Apply brake with PWM duty cycle
            int range = 1 << resolution;
            int duty = (int)(intensity * range);
            analogWrite(pin_fwd, duty);
            analogWrite(pin_rev, duty);
        }

    private:
        float setpoint = 0.0;
        int pin_fwd = 0;
        int pin_rev = 0;
        int frequency = 1000;
        int resolution = 10;

        void coast() {
            digitalWrite(pin_fwd, 0);
            digitalWrite(pin_rev, 0);
        }

        void forward(float rate, bool fast_decay = true) {
            int range = 1 << resolution;
            if(fast_decay) {
                analogWrite(pin_rev, rate*range);
                digitalWrite(pin_fwd, 0);
            } else {
                digitalWrite(pin_rev, 1);
                analogWrite(pin_fwd, (1-rate)*range);
            }


        }

        void reverse(float rate, bool fast_decay = true) {
            int range = 1 << resolution;
            if(fast_decay) {
                analogWrite(pin_fwd, rate*range);
                digitalWrite(pin_rev, 0);
            } else {
                digitalWrite(pin_fwd, 1);
                analogWrite(pin_rev, (1-rate)*range);
            }
        }

};