package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.hardware.AutoShootingController;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * Test program that FULLY AUTOMATICALLY shoots when robot is 3 feet from goal
 * NO BUTTONS NEEDED - just drive to 3 feet and it shoots!
 * 
 * Controls:
 * - Right Stick: Drive
 * - Left Stick X: Rotate
 * - B Button: Reset (allows re-shooting after completing)
 */
@Config
@TeleOp(name = "3 Feet Shoot Test")
public class ThreeFeetShootTest extends OpMode {
    
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;
    
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx intakefront, intakeback;
    private Servo launchgate;
    
    private AutoShootingController shootingController;
    
    // Distance threshold
    public static double SHOOT_DISTANCE_FEET = 3.0;
    public static double DISTANCE_TOLERANCE_FEET = 0.3;  // ±0.3 feet = ±3.6 inches
    
    // Auto-shoot state
    private boolean shooting = false;
    private boolean hasShot = false;  // Prevent repeated shooting
    private int shootState = 0;
    private ElapsedTime shootTimer;
    
    private boolean lastB = false;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(25);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize PedroPathing localization
        poseUpdater = new PoseUpdater(hardwareMap, FConstants.class, LConstants.class);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        
        // Initialize drive motors
        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");
        
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Initialize intake and launcher
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback = hardwareMap.get(DcMotorEx.class, "intakeback");
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        
        intakefront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeback.setDirection(DcMotorSimple.Direction.REVERSE);
        
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        // Initialize shooting controller
        shootingController = new AutoShootingController(hardwareMap);
        
        // Initialize timer
        shootTimer = new ElapsedTime();
        
        // Set launch gate to safe position
        launchgate.setPosition(0.5);
        
        telemetryA.addLine("3 Feet Shoot Test Initialized");
        telemetryA.addLine("=============================");
        telemetryA.addLine("FULLY AUTOMATIC SHOOTING!");
        telemetryA.addLine("Just drive to 3 feet from goal");
        telemetryA.addLine("");
        telemetryA.addLine("Controls:");
        telemetryA.addLine("  Right Stick = Drive");
        telemetryA.addLine("  Left Stick X = Rotate");
        telemetryA.addLine("  B Button = Reset (shoot again)");
        telemetryA.addData("Target Distance", "%.1f feet", SHOOT_DISTANCE_FEET);
        telemetryA.update();
    }
    
    @Override
    public void loop() {
        // Update localization
        poseUpdater.update();
        Pose currentPose = poseUpdater.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();
        
        // Update dashboard
        dashboardPoseTracker.update();
        Drawing.drawRobot(currentPose, "#4CAF50");
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50FF");
        Drawing.sendPacket();
        
        // Calculate distance to goal
        double deltaX = AutoShootingController.goalZoneX - currentX;
        double deltaY = AutoShootingController.goalZoneY - currentY;
        double distanceInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double distanceFeet = distanceInches / 12.0;
        
        // Check if within shooting distance
        boolean inShootingRange = Math.abs(distanceFeet - SHOOT_DISTANCE_FEET) <= DISTANCE_TOLERANCE_FEET;
        
        // ==================== DRIVE CONTROLS ====================
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x * 1.1;
        double rx = gamepad1.left_stick_x;
        
        double scale = 1.0;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        
        double flPower = (y + x + rx) / denominator * scale;
        double blPower = (y - x + rx) / denominator * scale;
        double frPower = (y - x - rx) / denominator * scale;
        double brPower = (y + x - rx) / denominator * scale;
        
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        
        // ==================== FULLY AUTOMATIC SHOOT CONTROL ====================
        boolean currentB = gamepad1.b;
        
        // B button resets the shot flag (allows re-shooting)
        if (currentB && !lastB) {
            hasShot = false;
            shooting = false;
            shootState = 0;
        }
        lastB = currentB;
        
        // AUTOMATICALLY start shooting if in range and haven't shot yet
        if (inShootingRange && !hasShot && !shooting) {
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }
        
        // Run shooting state machine
        String shootStatus = "Ready";
        if (shooting) {
            switch (shootState) {
                case 0: // Spin up shooter
                    shootStatus = "Spinning up...";
                    if (shootTimer.seconds() > 1.0) {
                        shootState = 1;
                        shootTimer.reset();
                    }
                    break;
                    
                case 1: // Start intakes
                    shootStatus = "Starting intakes...";
                    intakefront.setPower(1.0);
                    intakeback.setPower(1.0);
                    if (shootTimer.seconds() > 0.1) {
                        shootState = 2;
                        shootTimer.reset();
                    }
                    break;
                    
                case 2: // Fire shot
                    shootStatus = "Firing!";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.3) {
                        shootState = 3;
                        shootTimer.reset();
                    }
                    break;
                    
                case 3: // Reset gate and stop
                    shootStatus = "Complete!";
                    launchgate.setPosition(0.5);
                    intakefront.setPower(0);
                    intakeback.setPower(0);
                    if (shootTimer.seconds() > 0.2) {
                        shooting = false;
                        shootState = 0;
                        hasShot = true;  // Mark as shot (prevents re-shooting)
                    }
                    break;
            }
        }
        
        // ==================== SHOOTER VELOCITY CONTROL ====================
        // Shooter runs when: (1) actively shooting, OR (2) in range and haven't shot yet (pre-spin)
        boolean shooterOn = shooting || (inShootingRange && !hasShot);
        AutoShootingController.ShooterStatus status = 
            shootingController.update(shooterOn, currentX, currentY);
        
        // ==================== TELEMETRY ====================
        telemetryA.addLine("=== 3 FEET SHOOT TEST ===");
        telemetryA.addData("Position", String.format("(%.1f, %.1f) @ %.1f°", 
            currentX, currentY, Math.toDegrees(currentHeading)));
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== AUTO-SHOOT STATUS ===");
        telemetryA.addData("Has Shot", hasShot ? "YES (press B to reset)" : "No - Ready!");
        telemetryA.addData("Shooting", shooting ? "YES 🔥" : "No");
        if (shooting) {
            telemetryA.addData("Status", shootStatus);
            telemetryA.addData("State", shootState);
        }
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== DISTANCE CHECK ===");
        telemetryA.addData("Current Distance", "%.2f feet", distanceFeet);
        telemetryA.addData("Target Distance", "%.1f ± %.1f feet", SHOOT_DISTANCE_FEET, DISTANCE_TOLERANCE_FEET);
        telemetryA.addData("In Range", inShootingRange ? "YES ✓" : "No");
        telemetryA.addData("Distance Error", "%.2f feet", Math.abs(distanceFeet - SHOOT_DISTANCE_FEET));
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== SHOOTER VELOCITY ===");
        telemetryA.addData("Shooter", shooterOn ? "RUNNING" : "STOPPED");
        telemetryA.addData("At Target Speed", status.atTargetSpeed ? "✓ YES" : "No");
        telemetryA.addData("Target RPM", "%.0f", status.targetRPM);
        telemetryA.addData("Current RPM", "%.0f", status.avgRPM);
        telemetryA.addData("Error RPM", "%.0f", status.getErrorRPM());
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== CONTROLS ===");
        telemetryA.addLine("Just drive to 3 feet - it shoots automatically!");
        telemetryA.addData("B Button", "Reset (shoot again)");
        telemetryA.addLine("Right Stick = Drive | Left Stick X = Rotate");
        
        telemetryA.update();
    }
    
    @Override
    public void stop() {
        shootingController.stop();
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        intakefront.setPower(0);
        intakeback.setPower(0);
        launchgate.setPosition(0.5);
    }
}

