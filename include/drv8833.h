// controls one side of a drv8833 motor controller
//
// see https://www.ti.com/lit/ds/symlink/drv8833.pdf

#include "Arduino.h"
#include "utils.h"

class DRV8833 {
    public:
        void init(int pin_fwd , int pin_rev, int frequency = 1000, int resolution = 10) {
            this->pin_fwd = pin_fwd;
            this->pin_rev = pin_rev;
            this->frequency = frequency;
            this->resolution = resolution;
            pinMode(pin_fwd, OUTPUT);
            pinMode(pin_rev, OUTPUT);
            analogWriteFrequency(frequency);
            analogWriteResolution(resolution);
        }

        // rate is [-1.0,1.0], fast_decay will cause faster
        // deceleration
        void go(float rate, bool fast_decay = true) {
            pinMode(pin_fwd, OUTPUT);
            pinMode(pin_rev, OUTPUT);
            rate = clamp(rate, -1.0f, 1.0f);

            if (rate > 0.0) {
                forward(rate, fast_decay);
                return;
            }
            else if (rate < 0.0) {
                reverse(fabs(rate), fast_decay);
                return;
            }
            if (fast_decay) {
                brake();
            }
            coast();
        }

    private:
        int pin_fwd = 0;
        int pin_rev = 0;
        int frequency = 1000;
        int resolution = 10;

        void coast() {
            digitalWrite(pin_fwd, 0);
            digitalWrite(pin_rev, 0);
        }

        void brake() {
            digitalWrite(pin_fwd, 1);
            digitalWrite(pin_rev, 1);
        }

        void forward(float rate, bool fast_decay = true) {
            int range = 1 << resolution;
            analogWrite(pin_fwd, rate*range);
            digitalWrite(pin_rev, fast_decay ? 1 : 0);
        }

        void reverse(float rate, bool fast_decay = true) {
            int range = 1 << resolution;
            analogWrite(pin_rev, rate*range);
            digitalWrite(pin_fwd, fast_decay ? 1 : 0);
        }

};