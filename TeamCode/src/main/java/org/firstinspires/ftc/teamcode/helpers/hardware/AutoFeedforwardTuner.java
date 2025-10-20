package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Automatic Feedforward Tuner for FTC
 * 
 * This OpMode automatically calculates feedforward gains (kS and kV) by:
 * 1. Running motor at different power levels
 * 2. Recording steady-state velocities
 * 3. Using linear regression to calculate kS (intercept) and kV (slope)
 * 
 * Instructions:
 * 1. Configure MOTOR_NAME to match your hardware config
 * 2. Set REVERSE_MOTOR if needed
 * 3. Adjust TEST_POWERS array if needed (default tests 0.2 to 0.9)
 * 4. Run this OpMode (it's Autonomous)
 * 5. Wait for completion (~30-60 seconds)
 * 6. Results shown on telemetry and FTC Dashboard
 * 
 * The motor will spin at different speeds - ensure it's safe to run!
 */
@Config
@Autonomous(name = "Auto Feedforward Tuner", group = "Tuning")
public class AutoFeedforwardTuner extends LinearOpMode {

    // ===== CONFIGURATION =====
    public static String MOTOR_NAME = "motor";
    public static boolean REVERSE_MOTOR = false;
    
    // Test parameters
    public static double[] TEST_POWERS = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    public static double SETTLE_TIME_MS = 1500;  // Time to wait for velocity to stabilize
    public static double SAMPLE_TIME_MS = 1000;   // Time to sample stable velocity
    public static double VOLTAGE = 12.0;          // Nominal battery voltage
    
    // Stability criteria
    public static double VELOCITY_TOLERANCE = 50.0; // Max velocity variation for "stable"
    public static int MIN_STABLE_SAMPLES = 5;       // Minimum samples needed
    
    // ===== END CONFIGURATION =====

    private DcMotorEx motor;
    private List<DataPoint> dataPoints = new ArrayList<>();
    
    private static class DataPoint {
        double voltage;      // Applied voltage
        double velocity;     // Measured velocity (ticks/sec)
        double power;        // Applied power
        
        public DataPoint(double power, double voltage, double velocity) {
            this.power = power;
            this.voltage = voltage;
            this.velocity = velocity;
        }
    }
    
    private static class RegressionResult {
        double kS;        // Static friction (voltage)
        double kV;        // Velocity gain (volts per tick/sec)
        double rSquared;  // Goodness of fit (0-1, higher is better)
        
        public RegressionResult(double kS, double kV, double rSquared) {
            this.kS = kS;
            this.kV = kV;
            this.rSquared = rSquared;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize motor
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        if (REVERSE_MOTOR) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        telemetry.addLine("========================================");
        telemetry.addLine("  AUTOMATIC FEEDFORWARD TUNER");
        telemetry.addLine("========================================");
        telemetry.addLine();
        telemetry.addData("Motor", MOTOR_NAME);
        telemetry.addData("Test Points", TEST_POWERS.length);
        telemetry.addData("Total Time", "~" + (TEST_POWERS.length * (SETTLE_TIME_MS + SAMPLE_TIME_MS) / 1000) + " seconds");
        telemetry.addLine();
        telemetry.addLine("WARNING: Motor will spin!");
        telemetry.addLine("Ensure mechanism is safe to run.");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start auto-tuning...");
        telemetry.update();
        
        waitForStart();
        
        if (!opModeIsActive()) return;
        
        // Run characterization tests
        telemetry.addLine("Starting characterization...");
        telemetry.update();
        sleep(500);
        
        for (int i = 0; i < TEST_POWERS.length && opModeIsActive(); i++) {
            double testPower = TEST_POWERS[i];
            
            telemetry.clear();
            telemetry.addLine("========================================");
            telemetry.addData("Test Progress", "%d / %d", i + 1, TEST_POWERS.length);
            telemetry.addLine("========================================");
            telemetry.addData("Testing Power", "%.2f", testPower);
            telemetry.addLine();
            
            // Run test at this power level
            DataPoint result = runTest(testPower);
            
            if (result != null) {
                dataPoints.add(result);
                telemetry.addData("✓ Velocity", "%.0f ticks/sec", result.velocity);
                telemetry.addData("✓ Voltage", "%.2f V", result.voltage);
            } else {
                telemetry.addLine("✗ Test failed (unstable)");
            }
            
            telemetry.update();
            sleep(500);
        }
        
        // Stop motor
        motor.setPower(0);
        
        // Calculate feedforward gains
        telemetry.clear();
        telemetry.addLine("========================================");
        telemetry.addLine("  CALCULATING GAINS...");
        telemetry.addLine("========================================");
        telemetry.update();
        sleep(1000);
        
        if (dataPoints.size() < 3) {
            telemetry.clear();
            telemetry.addLine("ERROR: Not enough valid data points!");
            telemetry.addData("Collected", dataPoints.size());
            telemetry.addData("Required", "at least 3");
            telemetry.addLine();
            telemetry.addLine("Try adjusting TEST_POWERS or");
            telemetry.addLine("VELOCITY_TOLERANCE settings.");
            telemetry.update();
            sleep(5000);
            return;
        }
        
        RegressionResult result = calculateFeedforward();
        
        // Display results
        displayResults(result);
        
        // Keep results displayed
        while (opModeIsActive()) {
            sleep(100);
        }
    }
    
    /**
     * Run a single characterization test at specified power
     */
    private DataPoint runTest(double power) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        
        // Apply power
        motor.setPower(power);
        
        // Phase 1: Wait for settling
        telemetry.addData("Phase", "Settling... (%.1f s)", SETTLE_TIME_MS / 1000.0);
        telemetry.update();
        
        timer.reset();
        while (timer.milliseconds() < SETTLE_TIME_MS && opModeIsActive()) {
            double currentVel = motor.getVelocity();
            telemetry.addData("Current Velocity", "%.0f ticks/sec", currentVel);
            telemetry.update();
            sleep(50);
        }
        
        if (!opModeIsActive()) return null;
        
        // Phase 2: Sample stable velocity
        telemetry.addData("Phase", "Sampling... (%.1f s)", SAMPLE_TIME_MS / 1000.0);
        telemetry.update();
        
        List<Double> samples = new ArrayList<>();
        timer.reset();
        
        while (timer.milliseconds() < SAMPLE_TIME_MS && opModeIsActive()) {
            double velocity = motor.getVelocity();
            samples.add(velocity);
            
            telemetry.addData("Samples Collected", samples.size());
            telemetry.addData("Current Velocity", "%.0f ticks/sec", velocity);
            telemetry.update();
            
            sleep(50);
        }
        
        if (!opModeIsActive() || samples.isEmpty()) return null;
        
        // Check stability
        double avgVelocity = samples.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        double maxVelocity = samples.stream().mapToDouble(Double::doubleValue).max().orElse(0);
        double minVelocity = samples.stream().mapToDouble(Double::doubleValue).min().orElse(0);
        double variation = maxVelocity - minVelocity;
        
        boolean stable = variation < VELOCITY_TOLERANCE && samples.size() >= MIN_STABLE_SAMPLES;
        
        telemetry.addData("Stability Check", stable ? "PASS" : "FAIL");
        telemetry.addData("Variation", "%.1f ticks/sec", variation);
        telemetry.update();
        
        if (!stable) {
            return null;
        }
        
        double voltage = power * VOLTAGE;
        return new DataPoint(power, voltage, avgVelocity);
    }
    
    /**
     * Calculate feedforward gains using linear regression
     * Model: velocity = (voltage - kS) / kV
     * Rearranged: voltage = kS + kV * velocity
     */
    private RegressionResult calculateFeedforward() {
        int n = dataPoints.size();
        
        // Calculate means
        double meanVelocity = dataPoints.stream().mapToDouble(d -> d.velocity).average().orElse(0);
        double meanVoltage = dataPoints.stream().mapToDouble(d -> d.voltage).average().orElse(0);
        
        // Calculate kV (slope) and kS (intercept)
        double numerator = 0;
        double denominator = 0;
        
        for (DataPoint point : dataPoints) {
            numerator += (point.velocity - meanVelocity) * (point.voltage - meanVoltage);
            denominator += (point.velocity - meanVelocity) * (point.velocity - meanVelocity);
        }
        
        double kV = numerator / denominator;
        double kS = meanVoltage - kV * meanVelocity;
        
        // Calculate R² (goodness of fit)
        double totalSS = 0;
        double residualSS = 0;
        
        for (DataPoint point : dataPoints) {
            double predicted = kS + kV * point.velocity;
            totalSS += Math.pow(point.voltage - meanVoltage, 2);
            residualSS += Math.pow(point.voltage - predicted, 2);
        }
        
        double rSquared = 1.0 - (residualSS / totalSS);
        
        return new RegressionResult(kS, kV, rSquared);
    }
    
    /**
     * Display final results
     */
    private void displayResults(RegressionResult result) {
        telemetry.clear();
        telemetry.addLine("========================================");
        telemetry.addLine("  AUTO-TUNING COMPLETE!");
        telemetry.addLine("========================================");
        telemetry.addLine();
        
        telemetry.addLine("=== FEEDFORWARD GAINS ===");
        telemetry.addData("kS (static)", "%.6f V", result.kS);
        telemetry.addData("kV (velocity)", "%.8f V/(tick/s)", result.kV);
        telemetry.addData("kA (accel)", "0.0 (not tested)");
        telemetry.addLine();
        
        telemetry.addLine("=== MODEL QUALITY ===");
        telemetry.addData("R² (goodness of fit)", "%.4f", result.rSquared);
        if (result.rSquared > 0.95) {
            telemetry.addLine("✓ EXCELLENT fit!");
        } else if (result.rSquared > 0.90) {
            telemetry.addLine("✓ GOOD fit");
        } else if (result.rSquared > 0.80) {
            telemetry.addLine("⚠ ACCEPTABLE fit");
        } else {
            telemetry.addLine("✗ POOR fit - rerun test");
        }
        telemetry.addLine();
        
        telemetry.addLine("=== DATA POINTS ===");
        telemetry.addData("Valid Samples", dataPoints.size());
        for (int i = 0; i < dataPoints.size(); i++) {
            DataPoint p = dataPoints.get(i);
            double predicted = result.kS + result.kV * p.velocity;
            double error = Math.abs(p.voltage - predicted);
            telemetry.addData(String.format("Point %d", i + 1),
                    "V=%.0f, P=%.2f, Pred=%.2f (err: %.3f)",
                    p.velocity, p.voltage, predicted, error);
        }
        telemetry.addLine();
        
        telemetry.addLine("=== USAGE IN CODE ===");
        telemetry.addLine("Copy these values:");
        telemetry.addLine(String.format("kS = %.6f;", result.kS));
        telemetry.addLine(String.format("kV = %.8f;", result.kV));
        telemetry.addLine("kA = 0.0;");
        telemetry.addLine();
        
        telemetry.addLine("=== EXAMPLE CODE ===");
        telemetry.addLine("double ff = kS * sign(targetVel)");
        telemetry.addLine("          + kV * targetVel;");
        telemetry.addLine("motor.setPower(ff / 12.0);");
        telemetry.addLine();
        
        telemetry.addLine("Connect to FTC Dashboard for graphs!");
        telemetry.addLine("(192.168.43.1:8080/dash)");
        telemetry.update();
        
        // Also send to dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.getTelemetry().addLine("=== AUTO-TUNED GAINS ===");
        dashboard.getTelemetry().addData("kS", result.kS);
        dashboard.getTelemetry().addData("kV", result.kV);
        dashboard.getTelemetry().addData("R²", result.rSquared);
        dashboard.getTelemetry().update();
    }
}

