# Shooter Velocity PID Tuner - Usage Guide

## Quick Start

### 1. Setup FTC Dashboard
- Make sure FTC Dashboard is running on your computer
- Connect your robot and computer to the same WiFi network
- Open FTC Dashboard in browser (usually http://192.168.49.1:8080/dash)

### 2. Run the OpMode
1. Build and deploy your code to the robot
2. On the Driver Station, navigate to: **TeleOp** → **Tuning** → **Shooter Velocity PID Tuner**
3. Press **INIT** and then **START**

### 3. Gamepad Controls

| Button | Action |
|--------|--------|
| **A** | Toggle shooter ON/OFF |
| **B** | Reset PID integral terms (clears accumulated error) |
| **X** | Toggle individual motor mode (shared vs individual PID tuning) |
| **Y** | Stop all motors and reset statistics |
| **D-pad UP** | Increase target RPM by 50 |
| **D-pad DOWN** | Decrease target RPM by 50 |
| **Right Trigger** | Hold for low-power mode (uses LOW_POWER_RPM) |

## Tuning Workflow

### Step 1: Tune Feedforward First (kV)
1. Set all PID terms to zero: `kP = 0`, `kI = 0`, `kD = 0`, `kF = 0`
2. Set `kS = 0` (static friction, usually not needed)
3. Set a target RPM (e.g., 1100 RPM)
4. Press **A** to start the shooter
5. Adjust `kV` in FTC Dashboard until the shooter reaches approximately the target RPM
   - If actual RPM < target: increase `kV`
   - If actual RPM > target: decrease `kV`
6. Fine-tune `kV` to minimize steady-state error

### Step 2: Add Proportional (kP)
1. Increase `kP` gradually (start with 0.001)
2. Watch the error decrease
3. Stop when you see oscillations or overshoot
4. Reduce `kP` slightly if you see oscillations

### Step 3: Add Derivative (kD)
1. If you see oscillations, add `kD` (start with 0.0001)
2. Increase `kD` until oscillations are damped
3. Don't make `kD` too high (causes jitter)

### Step 4: Add Integral (kI) - Optional
1. Only add `kI` if there's persistent steady-state error after tuning kP and kD
2. Start very small (0.00001)
3. Increase gradually if needed
4. Use `I_ZONE` to limit when integral accumulates (typically 250-500)

### Step 5: Fine-tune kF (Feedforward in PIDF)
1. Some teams use `kF` in addition to `kV`
2. If using PIDF's built-in feedforward, set `kF` to complement `kV`
3. Usually `kF` is small (0.0005-0.001)

## What to Look For

### Good Tuning Indicators:
- ✅ Error stays within ±20-50 RPM of target
- ✅ Smooth response (no oscillations)
- ✅ Fast response time (reaches target quickly)
- ✅ Minimal overshoot
- ✅ Consistent performance at different RPMs

### Bad Tuning Indicators:
- ❌ Oscillations around target (kP too high, needs kD)
- ❌ Slow response (kP too low)
- ❌ Large steady-state error (needs kI or better kV)
- ❌ Overshoot (kP too high, or needs kD)
- ❌ Jittery behavior (kD too high)

## Reading the Telemetry

### Driver Station Screen:
- **Target RPM**: What you're trying to achieve
- **Actual RPM**: Current shooter speed
- **Error**: Difference between target and actual
- **Power**: Motor power being applied
- **Statistics**: Max RPM, min error, average error since last reset

### FTC Dashboard:
- All PID coefficients can be adjusted in real-time
- Changes apply immediately (no need to restart)
- Use the config section to modify values

## Tuning Tips

1. **Start with feedforward (kV)** - This does most of the work
2. **Tune at your typical shooting RPM** - Usually around 1100-1400 RPM
3. **Test at multiple RPMs** - Good tuning should work across your RPM range
4. **Use voltage compensation** - Keep `USE_VOLTAGE_COMPENSATION = true`
5. **Enable overshoot protection** - Keep `USE_OVERSPOOT_PROTECTION = true`
6. **Individual mode** - Only use if motors behave very differently
7. **Reset integrals** - Press **B** if integral term builds up incorrectly

## Example Starting Values

Based on your current code:
- `kP = 0.0015`
- `kI = 0.0` (usually not needed)
- `kD = 0.0000` (add if oscillations)
- `kF = 0.0005`
- `kV = 0.0005`
- `kS = 0.0`
- `I_ZONE = 250.0`

## Troubleshooting

**Shooter won't start:**
- Check motor connections in config
- Verify motor names match your configuration ("shootr", "shootl")

**Can't reach target RPM:**
- Increase `kV` (feedforward)
- Check battery voltage (should be >12V)
- Check for mechanical issues

**Oscillations:**
- Reduce `kP`
- Add `kD` (start small: 0.0001)
- Make sure `kI = 0` if not needed

**Slow response:**
- Increase `kP`
- Increase `kV` if steady-state error is large

**Motors spinning at different speeds:**
- Enable individual mode (press **X**)
- Tune each motor separately

## Saving Your Values

After tuning, copy the values from FTC Dashboard and update them in:
- Your teleop opmodes (e.g., `CosmobotsBlueTeleop.java`)
- Your auto opmodes that use shooting
- Any shooter helper classes

Good values to save:
- `kP`, `kI`, `kD`, `kF`
- `kV`, `kS`
- `I_ZONE`
