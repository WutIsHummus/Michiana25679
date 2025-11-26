package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter Velocity Tuner - Based on FTC-23511's LaunchMotorTuner pattern
 * Uses PIDFController with voltage compensation for consistent velocity control
 * 
 * @author Based on FTC-23511/Decode-2026 LaunchMotorTuner
 */
@Config
@TeleOp(name = "Shooter Velocity Tuner (FTC-23511)", group = "Tuning")
public class ShooterVelocityTunerFTC23511 extends LinearOpMode {
    
    // Motor Constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;
    
    // PIDF Constants (based on FTC-23511 pattern)
    public static double P = 0.0015;
    public static double I = 0.0;
    public static double D = 0.000;
    public static double F = 0.0009;
    public static double I_ZONE = 250.0;  // Integrator clamp range
    
    public static double TARGET_RPM = 2000.0;  // Target velocity in RPM (for tuning)
    public static double POS_TOLERANCE = 0;  // Position tolerance (not used for velocity)
    
    // Maximum velocity limit (safety check)
    public static double MAX_VELOCITY = 5000.0;
    
    private DcMotorEx shoot;
    private DcMotorEx shootL;
    
    // PIDF Controller for shooter (based on FTC-23511 pattern - single controller for average velocity)
    private PIDFController shooterPIDF;
    
    // Velocity tracking (with limits like FTC-23511)
    private double motorVel = 0;
    private double lastError = 0;
    private double integralSum = 0;
    
    // PIDF output tracking for graphing
    private double pidfOutput = 0;
    private double pidfP = 0;
    private double pidfI = 0;
    private double pidfD = 0;
    private double pidfF = 0;
    
    private FtcDashboard dashboard;
    private ElapsedTime timer;
    private VoltageSensor voltageSensor;
    
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        timer = new ElapsedTime();
        
        initializeMotors();
        
        // Get voltage sensor (for voltage compensation like FTC-23511)
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        
        telemetry.addLine("Shooter Velocity Tuner (FTC-23511 Pattern)");
        telemetry.addLine("Use FTC Dashboard to:");
        telemetry.addLine("- Adjust TARGET_RPM");
        telemetry.addLine("- Tune P, I, D, F");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Motors start automatically!");
        telemetry.addLine("B: Stop shooter");
        telemetry.addLine("X: Measure ticks/revolution");
        telemetry.update();
        
        waitForStart();
        timer.reset();
        
        while (opModeIsActive()) {
            // Automatically run motors (no button press needed)
            if (TARGET_RPM <= 0) {
                // Warn user if TARGET_RPM is not set
                telemetry.addLine("⚠️ WARNING: TARGET_RPM is 0 or negative!");
                telemetry.addLine("Set TARGET_RPM in FTC Dashboard first!");
                telemetry.update();
                sleep(100);
            } else {
                runMotorsWithPIDF();
            }
            
            // Button controls (optional - motors run automatically)
            if (gamepad1.b) {
                stopAllMotors();
            } else if (gamepad1.x) {
                measureTicksPerRevolution();
            }
            
            // Always update telemetry for consistent graphing (except during tick measurement)
            if (!gamepad1.x) {
                updateTelemetry();
            }
            sleep(20);
        }
        
        stopAllMotors();
        
        // Log final PIDF values (like FTC-23511)
        Log.v("P", String.valueOf(P));
        Log.v("I", String.valueOf(I));
        Log.v("D", String.valueOf(D));
        Log.v("F", String.valueOf(F));
        Log.v("TARGET_RPM", String.valueOf(TARGET_RPM));
    }
    
    /**
     * Convert RPM to ticks per second
     */
    private static double rpmToTicksPerSec(double rpm) {
        double motorRPM = rpm * GEAR_RATIO;
        return (motorRPM / 60.0) * TICKS_PER_REV;
    }
    
    /**
     * Convert ticks per second to RPM
     */
    private static double ticksPerSecToRPM(double tps) {
        double motorRPM = (tps / TICKS_PER_REV) * 60.0;
        return motorRPM / GEAR_RATIO;
    }
    
    private void initializeMotors() {
        shoot = hardwareMap.get(DcMotorEx.class, "shootr");
        shootL = hardwareMap.get(DcMotorEx.class, "shootl");
        
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set shooter motor directions (matching teleop configuration)
        shootL.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set velocity PIDF coefficients directly on motors (for velocity control mode)
        shoot.setVelocityPIDFCoefficients(P, I, D, F);
        shootL.setVelocityPIDFCoefficients(P, I, D, F);
        
        // Initialize PIDF controller (based on FTC-23511 pattern - single controller for average velocity)
        shooterPIDF = new PIDFController(P, I, D, F);
        shooterPIDF.setTolerance(POS_TOLERANCE, 0);
        shooterPIDF.setIntegrationBounds(-I_ZONE, I_ZONE);
    }
    
    private void runMotorsWithPIDF() {
        // Get voltage for compensation (based on FTC-23511 pattern)
        double voltage = voltageSensor.getVoltage();
        
        // Convert target RPM to ticks per second
        double targetTPS = rpmToTicksPerSec(TARGET_RPM);
        
        // Get current velocity from shootr (only motor with encoder)
        double vR = shoot.getVelocity();
        
        // Always update motorVel with current velocity (unless it's an erroneous reading)
        if (Math.abs(vR) < MAX_VELOCITY) {
            motorVel = vR;
        }
        
        // Update velocity PIDF coefficients on motors with voltage compensation (F / (voltage / 12.0))
        double compensatedF = F / (voltage / 12.0);
        shoot.setVelocityPIDFCoefficients(P, I, D, compensatedF);
        shootL.setVelocityPIDFCoefficients(P, I, D, compensatedF);
        
        // Update PIDF controller for calculation (must update every loop!)
        shooterPIDF.setPIDF(P, I, D, compensatedF);
        shooterPIDF.setTolerance(POS_TOLERANCE, 0);
        shooterPIDF.setIntegrationBounds(-I_ZONE, I_ZONE);
        shooterPIDF.setSetPoint(targetTPS);
        
        // Calculate error for PIDF components
        double error = targetTPS - motorVel;
        double derivative = error - lastError;
        
        // Calculate PIDF components manually for graphing
        pidfP = P * error;
        
        // Integral term (with clamping using I_ZONE)
        integralSum += error;
        if (Math.abs(integralSum) > I_ZONE) {
            integralSum = Math.signum(integralSum) * I_ZONE;
        }
        pidfI = I * integralSum;
        
        // Derivative term
        pidfD = D * derivative;
        
        // Feedforward term (this is critical - should provide base power)
        pidfF = compensatedF * targetTPS;
        
        // Total PIDF output
        pidfOutput = pidfP + pidfI + pidfD + pidfF;
        
        // Use PIDFController for actual control (it handles integration bounds properly)
        double power = shooterPIDF.calculate(motorVel, targetTPS);
        
        // Clamp power to valid range
        power = Math.max(-1.0, Math.min(1.0, power));
        
        // Update last error for next iteration
        lastError = error;
        
        // Apply same power to both motors (like FTC-23511's robot.launchMotors.set(power))
        shoot.setPower(power);
        shootL.setPower(power);
        
        // Debug: Log if power is very small
        if (Math.abs(power) < 0.01 && TARGET_RPM > 0) {
            // This shouldn't happen if F term is correct
        }
    }
    
    private void stopAllMotors() {
        shoot.setPower(0);
        shootL.setPower(0);
        
        // Reset PIDF controller (like FTC-23511)
        if (shooterPIDF != null) shooterPIDF.reset();
        
        // Reset manual tracking variables
        integralSum = 0;
        lastError = 0;
    }
    
    private void measureTicksPerRevolution() {
        telemetry.addLine("MEASURING TICKS PER REVOLUTION");
        telemetry.addLine("Manually spin each shooter motor EXACTLY 1 revolution");
        telemetry.addLine("Press A when done");
        telemetry.update();
        
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        while (opModeIsActive() && !gamepad1.a) {
            telemetry.addData("Shoot (right)", shoot.getCurrentPosition());
            telemetry.addData("ShootL (left)", shootL.getCurrentPosition());
            telemetry.update();
        }
        
        telemetry.addLine("=== TICKS PER REVOLUTION ===");
        telemetry.addData("Shoot (right)", shoot.getCurrentPosition());
        telemetry.addData("ShootL (left)", shootL.getCurrentPosition());
        telemetry.addLine();
        telemetry.addLine("Press B to continue");
        telemetry.update();
        
        while (opModeIsActive() && !gamepad1.b) {
            sleep(50);
        }
        
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void updateTelemetry() {
        double voltage = voltageSensor.getVoltage();
        double compensatedF = F / (voltage / 12.0);
        
        // Get current velocity from shootr (always get fresh value)
        double vR = shoot.getVelocity();
        double vUsed = (Math.abs(vR) < MAX_VELOCITY) ? vR : motorVel;  // Use current or last valid
        
        // Convert to RPM for display (always show RPM, never ticks)
        double currentRPM = ticksPerSecToRPM(vUsed);
        double targetRPMDisplay = TARGET_RPM;
        
        // Calculate current error for display (even if not running)
        double targetTPS = rpmToTicksPerSec(TARGET_RPM);
        double currentError = targetTPS - vUsed;
        
        // IMPORTANT: These telemetry keys must be at the TOP and sent EVERY loop for graphing
        // FTC Dashboard graphs based on consistent key names - they must appear first!
        // Use raw numeric values (no formatting) for better graphing
        telemetry.addData("Target RPM", targetRPMDisplay);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Error RPM", targetRPMDisplay - currentRPM);
        telemetry.addData("PIDF Output", pidfOutput);
        telemetry.addData("PIDF P", pidfP);
        telemetry.addData("PIDF I", pidfI);
        telemetry.addData("PIDF D", pidfD);
        telemetry.addData("PIDF F", pidfF);
        telemetry.addData("Power Applied", shoot.getPower());
        
        telemetry.addLine();
        telemetry.addLine("=== PIDF CONSTANTS ===");
        telemetry.addData("P", "%.6f", P);
        telemetry.addData("I", "%.6f", I);
        telemetry.addData("D", "%.6f", D);
        telemetry.addData("F (base)", "%.6f", F);
        telemetry.addData("I_ZONE", "%.1f", I_ZONE);
        telemetry.addData("F (compensated)", "%.6f", compensatedF);
        telemetry.addData("Voltage", "%.2f V", voltage);
        telemetry.addLine();
        
        telemetry.addLine("=== VELOCITY INFO ===");
        telemetry.addData("Target (RPM)", "%.1f", targetRPMDisplay);
        telemetry.addData("Current (RPM)", "%.1f", currentRPM);
        telemetry.addData("Error (RPM)", "%.1f", targetRPMDisplay - currentRPM);
        if (TARGET_RPM <= 0) {
            telemetry.addLine("⚠️ SET TARGET_RPM IN FTC DASHBOARD!");
        }
        telemetry.addLine();
        
        telemetry.addLine("=== SHOOTER MOTORS ===");
        telemetry.addData("Power Applied", "%.4f", shoot.getPower());
        telemetry.addData("Target TPS", "%.1f", rpmToTicksPerSec(TARGET_RPM));
        telemetry.addData("Current TPS", "%.1f", vUsed);
        telemetry.addLine();
        
        telemetry.addLine("--- Right Motor (shootr) - HAS ENCODER ---");
        telemetry.addData("Velocity RPM", "%.1f", ticksPerSecToRPM(vR));
        telemetry.addData("Velocity TPS", "%.1f", vR);
        telemetry.addData("Position", shoot.getCurrentPosition());
        telemetry.addData("Power Set", "%.4f", shoot.getPower());
        
        telemetry.addLine("--- Left Motor (shootl) - NO ENCODER ---");
        telemetry.addData("Position", shootL.getCurrentPosition());
        telemetry.addData("Power Set", "%.4f", shootL.getPower());
        telemetry.addData("Note", "No encoder - follows shootr power");
        
        // Debug info
        if (Math.abs(shoot.getPower()) < 0.01 && TARGET_RPM > 100) {
            telemetry.addLine("⚠️ WARNING: Power is very low!");
            telemetry.addData("Check", "F term might be too small");
            telemetry.addData("F term", "%.6f", F);
            telemetry.addData("F*TargetTPS", "%.4f", F * rpmToTicksPerSec(TARGET_RPM));
        }
        
        telemetry.update();
    }
}

