# Wall Approach Braking — Status & Lessons Learned

**Date:** February 8, 2026  
**Committed checkpoint:** `de6d509` (physics-based braking curve, pre-refactor)  
**Uncommitted:** Architectural refactor of `update_twist_control()` — not yet working

---

## What Works (committed code)

Physics-based braking curve in `set_approach_twist()` achieves **5mm accuracy at 0.5 m/s**. It uses the characterized coast model (`decel = 0.951 + 0.794v`) to compute a velocity-for-distance curve and feeds explicit `(v_target, accel_target)` pairs to the PI controller. The PI's feedforward + acceleration terms do most of the work; PI correction handles tracking error.

## What Doesn't Work

**Speed = 0.8 m/s crashes into the wall.** A negative velocity reading (wheel bounce / tread ripple) fed into `v_current / v_brake` and produced a *positive* `accel_target`, telling the robot to speed up while braking. Fixed with `fabs()` but only in the uncommitted code.

**The committed `update_twist_control()` has structural problems:**
- A deadband early-return at `v_target ≈ 0` that bypasses the PI, causing discontinuities at the stop transition
- Per-wheel brake decision tree (~60 lines) with PI resets during braking that destroy integrator state
- Special-case coast-vs-brake threshold logic that doesn't compose with the PI

## The Refactor Attempt (uncommitted, ~4 hours of iteration)

**Goal:** Unified, modeless motor control — PI runs continuously, its voltage output maps to `go()` / `brake()` / `coast()` without mode switches or PI resets.

**Changes made:**
1. Removed deadband early-return — PI drives to zero naturally
2. Removed per-wheel brake decision tree and PI resets
3. Added unified voltage-to-motor lambda (`V>0 → go`, `V<0 → brake`, `V≈0 → coast`)
4. Zeroed `ff_offset` when `v_target ≈ 0` so stopped state doesn't command 0.271V forward
5. Added `fabs(v_current)` fix in braking ratio
6. Added NaN emergency stop for TOF sensor dropout

**Patches added trying to stabilize it (each fixing a regression from the last):**
- Voltage slew rate limit (0.8G max = 0.14V per 10ms step)
- Decel safety clamp V ≤ 0 → too aggressive, 47mm undershoot
- Revised clamp: V ≤ (ff_for_target + 0.5V margin) → built, not yet tested

## Test Scorecard (18 runs today)

| Run | Code State | Result | Failure Mode |
|-----|-----------|--------|-------------|
| Pre-refactor, v=0.5 | committed | **5mm** ✅ | — |
| Pre-refactor, v=0.8 | committed | **-179mm** 💥 | Negative velocity → positive accel |
| Post-refactor #1 | refactored | **crash** 💥 | TOF all NaN (I2C lockup, unrelated) |
| Post-refactor #2 | refactored | **crash** 💥 | TOF NaN mid-run |
| Post-refactor #3-5 | refactored | **-8 to -22mm**, jerky | PI oscillation at end of braking |
| +slew limit +decel clamp | refactored | **-191mm** 💥 | TOF NaN + oscillation |
| +V≤0 clamp | refactored | **47mm undershoot** | Clamp prevented tracking braking curve |
| +revised clamp | refactored | **not tested** | Build succeeded, awaiting upload |

**Pattern:** Each fix addressed one symptom but introduced a new one. The PI's high gains (`k_p=4.0, k_i=3.0`) amplify any disturbance, and layered clamps fight the PI rather than working with it.

## Key Physical Constraints

- **DRV8833 slow-decay PWM:** `go(sub-backEMF)` already provides electrical braking proportional to the gap. The driver is naturally "modeless" — the code should be too.
- **Slick floor + tank treads:** Real acceleration can approach 0.8G. Velocity readings are real, not noise. Traction bars cause velocity ripple at low speed.
- **TOF sensor:** Returns NaN below ~30mm and above ~1.3m. Losing distance feedback is a separate problem (I2C recovery) but must be handled safely.

## Architectural Takeaway

The refactor's *direction* is right (modeless, PI-continuous) but the *execution* failed because we kept the same aggressive PI gains and bolted on clamps/limits to compensate. The clamps fight the PI. A stepwise approach would be:

1. **Get the PI to track a simple velocity step** (cruise → zero) cleanly, with no clamps, no wall, just open floor. Tune gains for this.
2. **Add the braking curve planner** on top of a proven PI. If the PI can already track a ramp-down, the planner just shapes the ramp.
3. **Add wall proximity** (TOF feedback, safety stops) as a separate layer.

Each step should be testable independently. The mistake was changing the control architecture *and* testing it against the hardest scenario (wall approach at speed) simultaneously.
