package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * FTC Velocity Feedforward + PID Tuner
 * 
 * This OpMode helps you tune:
 * 1. Feedforward gains (kS, kV, kA) for velocity control
 * 2. PID gains (kP, kI, kD) for feedback correction
 * 
 * Instructions:
 * 1. Connect to FTC Dashboard (192.168.43.1:8080/dash or 192.168.49.1:8080/dash)
 * 2. Start with feedforward tuning:
 *    - Set kP, kI, kD to 0
 *    - Tune kS (static friction) - min voltage to start movement
 *    - Tune kV (velocity gain) - voltage per unit velocity
 *    - Tune kA (acceleration gain) - voltage per unit acceleration
 * 3. Then tune PID for correction:
 *    - Start with kP (proportional gain)
 *    - Add kD if needed (derivative gain)
 *    - Add kI last if steady-state error exists (integral gain)
 * 
 * Controls:
 * - Gamepad1 Left Stick Y: Manual motor control (override)
 * - Gamepad1 A: Toggle auto velocity control
 * - Gamepad1 Dpad Up/Down: Increase/Decrease target velocity
 */
@Config
@TeleOp(name = "Velocity Feedforward Tuner", group = "Tuning")
public class VelocityFeedforwardTuner extends OpMode {

    // ===== CONFIGURATION - Edit these =====
    public static String MOTOR_NAME = "motor"; // Change to your motor name in config
    public static boolean REVERSE_MOTOR = false;
    
    // Feedforward gains (tune these first with PID at 0)
    public static double kS = 0.0;  // Static friction (voltage to overcome stiction)
    public static double kV = 0.0;  // Velocity feedforward (volts per tick/sec)
    public static double kA = 0.0;  // Acceleration feedforward (volts per tick/sec²)
    
    // PID gains (tune these after feedforward)
    public static double kP = 0.0;  // Proportional gain
    public static double kI = 0.0;  // Integral gain
    public static double kD = 0.0;  // Derivative gain
    
    // Target velocity control
    public static double TARGET_VELOCITY = 1000.0;  // Target velocity in ticks/sec
    public static double VELOCITY_INCREMENT = 100.0; // Increment for dpad control
    public static double MAX_VELOCITY = 3000.0;      // Maximum target velocity
    
    // System parameters
    public static double MAX_VOLTAGE = 12.0;         // Battery voltage (nominal)
    public static double VELOCITY_TOLERANCE = 50.0;  // Acceptable velocity error
    
    // ===== END CONFIGURATION =====

    private DcMotorEx motor;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime controlLoopTimer = new ElapsedTime();
    
    // Control state
    private boolean autoControl = false;
    private boolean lastAButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    
    // Velocity control variables
    private double lastVelocity = 0.0;
    private double lastTime = 0.0;
    private double integralSum = 0.0;
    private double maxIntegral = 1.0;
    
    // Telemetry tracking
    private double averageError = 0.0;
    private double maxError = 0.0;
    private int sampleCount = 0;

    @Override
    public void init() {
        // Setup telemetry for FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize motor
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        if (REVERSE_MOTOR) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        telemetry.addLine("=================================");
        telemetry.addLine("  VELOCITY FEEDFORWARD TUNER");
        telemetry.addLine("=================================");
        telemetry.addLine();
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Connect to FTC Dashboard");
        telemetry.addLine("2. Tune kS, kV, kA (with PID=0)");
        telemetry.addLine("3. Then tune kP, kI, kD");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  A Button: Toggle auto control");
        telemetry.addLine("  Dpad Up/Down: Change velocity");
        telemetry.addLine("  Left Stick Y: Manual override");
        telemetry.addLine();
        telemetry.addLine("Ready to start!");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        controlLoopTimer.reset();
        lastTime = runtime.seconds();
    }

    @Override
    public void loop() {
        // ===== CONTROL INPUT =====
        
        // Toggle auto control with A button
        boolean currentA = gamepad1.a;
        if (currentA && !lastAButton) {
            autoControl = !autoControl;
            if (autoControl) {
                integralSum = 0; // Reset integral
                sampleCount = 0;
                averageError = 0;
                maxError = 0;
            }
        }
        lastAButton = currentA;
        
        // Adjust target velocity with dpad
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;
        
        if (currentDpadUp && !lastDpadUp) {
            TARGET_VELOCITY = Math.min(TARGET_VELOCITY + VELOCITY_INCREMENT, MAX_VELOCITY);
            integralSum = 0; // Reset integral on target change
        }
        if (currentDpadDown && !lastDpadDown) {
            TARGET_VELOCITY = Math.max(TARGET_VELOCITY - VELOCITY_INCREMENT, -MAX_VELOCITY);
            integralSum = 0; // Reset integral on target change
        }
        
        lastDpadUp = currentDpadUp;
        lastDpadDown = currentDpadDown;
        
        // ===== MOTOR CONTROL =====
        
        double motorPower;
        double currentVelocity = motor.getVelocity();
        double currentTime = runtime.seconds();
        double dt = currentTime - lastTime;
        
        if (dt <= 0.0) dt = 0.001; // Prevent division by zero
        
        if (autoControl) {
            // Feedforward + PID velocity control
            
            // Calculate acceleration
            double acceleration = (currentVelocity - lastVelocity) / dt;
            
            // Feedforward component
            double feedforward = kV * TARGET_VELOCITY + kA * acceleration;
            
            // Add static friction term (sign of target velocity)
            if (Math.abs(TARGET_VELOCITY) > 1.0) {
                feedforward += Math.copySign(kS, TARGET_VELOCITY);
            }
            
            // PID feedback component
            double error = TARGET_VELOCITY - currentVelocity;
            
            // Proportional
            double pTerm = kP * error;
            
            // Integral (with anti-windup)
            integralSum += error * dt;
            if (kI != 0) {
                integralSum = Math.max(-maxIntegral / kI, Math.min(integralSum, maxIntegral / kI));
            }
            double iTerm = kI * integralSum;
            
            // Derivative (on measurement to avoid derivative kick)
            double velocityDerivative = (currentVelocity - lastVelocity) / dt;
            double dTerm = -kD * velocityDerivative;
            
            // Total voltage
            double voltage = feedforward + pTerm + iTerm + dTerm;
            
            // Convert voltage to motor power [-1, 1]
            motorPower = voltage / MAX_VOLTAGE;
            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
            
            // Track error statistics
            sampleCount++;
            averageError = ((averageError * (sampleCount - 1)) + Math.abs(error)) / sampleCount;
            maxError = Math.max(maxError, Math.abs(error));
            
            // Telemetry for tuning
            telemetry.addLine("=== AUTO VELOCITY CONTROL ===");
            telemetry.addData("Target Velocity", "%.0f ticks/sec", TARGET_VELOCITY);
            telemetry.addData("Current Velocity", "%.0f ticks/sec", currentVelocity);
            telemetry.addData("Velocity Error", "%.0f ticks/sec", error);
            telemetry.addLine();
            
            telemetry.addLine("--- Feedforward ---");
            telemetry.addData("kS (static)", "%.4f V", kS);
            telemetry.addData("kV (velocity)", "%.6f V/(tick/s)", kV);
            telemetry.addData("kA (accel)", "%.6f V/(tick/s²)", kA);
            telemetry.addData("FF Output", "%.3f V", feedforward);
            telemetry.addLine();
            
            telemetry.addLine("--- PID Feedback ---");
            telemetry.addData("kP", "%.6f", kP);
            telemetry.addData("kI", "%.6f", kI);
            telemetry.addData("kD", "%.6f", kD);
            telemetry.addData("P Term", "%.3f V", pTerm);
            telemetry.addData("I Term", "%.3f V", iTerm);
            telemetry.addData("D Term", "%.3f V", dTerm);
            telemetry.addLine();
            
            telemetry.addLine("--- Output ---");
            telemetry.addData("Total Voltage", "%.3f V", voltage);
            telemetry.addData("Motor Power", "%.3f", motorPower);
            telemetry.addLine();
            
            telemetry.addLine("--- Performance ---");
            telemetry.addData("Avg Error", "%.1f ticks/sec", averageError);
            telemetry.addData("Max Error", "%.1f ticks/sec", maxError);
            telemetry.addData("Within Tolerance", (Math.abs(error) < VELOCITY_TOLERANCE) ? "YES" : "NO");
            
        } else {
            // Manual control with gamepad
            motorPower = -gamepad1.left_stick_y;
            integralSum = 0; // Reset integral in manual mode
            
            telemetry.addLine("=== MANUAL CONTROL ===");
            telemetry.addData("Current Velocity", "%.0f ticks/sec", currentVelocity);
            telemetry.addData("Motor Power", "%.3f", motorPower);
            telemetry.addLine();
            telemetry.addLine("Press A to enable auto control");
        }
        
        // Set motor power
        motor.setPower(motorPower);
        
        // Update for next iteration
        lastVelocity = currentVelocity;
        lastTime = currentTime;
        
        // ===== ADDITIONAL TELEMETRY =====
        
        telemetry.addLine();
        telemetry.addLine("--- Motor Info ---");
        telemetry.addData("Position", motor.getCurrentPosition());
        telemetry.addData("Current Draw", "%.2f A", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Loop Time", "%.1f ms", controlLoopTimer.milliseconds());
        controlLoopTimer.reset();
        
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addData("Auto Control", autoControl ? "ON (A to disable)" : "OFF (A to enable)");
        telemetry.addData("Target Velocity", "%.0f (Dpad ↑↓)", TARGET_VELOCITY);
        
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
        
        telemetry.addLine();
        telemetry.addLine("=================================");
        telemetry.addLine("  TUNING SESSION COMPLETE");
        telemetry.addLine("=================================");
        if (sampleCount > 0) {
            telemetry.addLine();
            telemetry.addLine("Final Statistics:");
            telemetry.addData("Average Error", "%.1f ticks/sec", averageError);
            telemetry.addData("Max Error", "%.1f ticks/sec", maxError);
            telemetry.addLine();
            telemetry.addLine("Current Gains:");
            telemetry.addData("kS", kS);
            telemetry.addData("kV", kV);
            telemetry.addData("kA", kA);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
        }
        telemetry.update();
    }
}

