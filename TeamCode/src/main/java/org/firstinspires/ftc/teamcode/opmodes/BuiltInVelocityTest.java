package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Tests the built-in FTC velocity control (RUN_USING_ENCODER mode)
 * This uses the motor controller's internal PID, not custom PID
 * 
 * Controls:
 * - A Button: Start shooter at target RPM
 * - B Button: Stop shooter
 * - Adjust targetRPM from FTC Dashboard
 */
@Config
@TeleOp(name = "Built-In Velocity Test")
public class BuiltInVelocityTest extends OpMode {
    
    private Telemetry telemetryA;
    
    // Shooter motors
    private DcMotorEx shootr, shootl;
    
    // Tunable parameters
    public static double targetRPM = 1475;  // Target RPM (adjustable from dashboard)
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;
    
    private boolean shooterOn = false;
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize shooter motors
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");
        
        // Set motor directions
        shootl.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // CRITICAL: Use RUN_USING_ENCODER for built-in velocity control
        shootr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        shootr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // RUN_USING_ENCODER mode enables built-in velocity PID
        shootr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetryA.addLine("Built-In Velocity Test Initialized");
        telemetryA.addLine("==================================");
        telemetryA.addLine("Uses motor controller's internal PID");
        telemetryA.addLine("No custom PID code");
        telemetryA.addLine("");
        telemetryA.addLine("Controls:");
        telemetryA.addLine("  A Button = Start shooter");
        telemetryA.addLine("  B Button = Stop shooter");
        telemetryA.addData("Target RPM", targetRPM);
        telemetryA.update();
    }

    @Override
    public void loop() {
        // Button controls
        boolean currentA = gamepad1.a;
        boolean currentB = gamepad1.b;
        
        if (currentA && !lastA) {
            shooterOn = true;
        }
        if (currentB && !lastB) {
            shooterOn = false;
        }
        lastA = currentA;
        lastB = currentB;
        
        // Convert RPM to ticks per second
        double targetTPS = rpmToTicksPerSec(targetRPM);
        
        if (shooterOn) {
            // Use built-in velocity control - setVelocity()
            // The motor controller's PID handles everything automatically!
            shootr.setVelocity(targetTPS);
            shootl.setVelocity(targetTPS);
        } else {
            shootr.setVelocity(0);
            shootl.setVelocity(0);
        }
        
        // Read actual velocities
        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double vAvg = 0.5 * (Math.abs(vR) + Math.abs(vL));
        
        // Convert to RPM
        double rpmR = ticksPerSecToRPM(vR);
        double rpmL = ticksPerSecToRPM(vL);
        double avgRPM = ticksPerSecToRPM(vAvg);
        
        // Calculate error
        double errorRPM = targetRPM - avgRPM;
        double errorPercent = (errorRPM / targetRPM) * 100.0;
        
        // Telemetry
        telemetryA.addLine("=== BUILT-IN VELOCITY CONTROL ===");
        telemetryA.addData("Shooter Status", shooterOn ? "RUNNING" : "STOPPED");
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== TARGET ===");
        telemetryA.addData("Target RPM", "%.0f", targetRPM);
        telemetryA.addData("Target TPS", "%.1f", targetTPS);
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== ACTUAL ===");
        telemetryA.addData("Avg RPM", "%.0f", avgRPM);
        telemetryA.addData("Right Motor RPM", "%.0f", rpmR);
        telemetryA.addData("Left Motor RPM", "%.0f", rpmL);
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== ERROR ===");
        telemetryA.addData("Error RPM", "%.0f", errorRPM);
        telemetryA.addData("Error %", "%.1f%%", errorPercent);
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== RAW VALUES ===");
        telemetryA.addData("Right Motor TPS", "%.1f", vR);
        telemetryA.addData("Left Motor TPS", "%.1f", vL);
        telemetryA.addData("Avg TPS", "%.1f", vAvg);
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== CONTROLS ===");
        telemetryA.addData("A Button", "Start shooter");
        telemetryA.addData("B Button", "Stop shooter");
        telemetryA.addLine("Adjust targetRPM from Dashboard");
        
        telemetryA.update();
    }

    @Override
    public void stop() {
        shootr.setVelocity(0);
        shootl.setVelocity(0);
        shootr.setPower(0);
        shootl.setPower(0);
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
}

