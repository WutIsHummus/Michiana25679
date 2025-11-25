package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
// TODO: PoseUpdater removed in PedroPathing 2.0
// import com.pedropathing.telemetry.PoseUpdater;
// import com.pedropathing.telemetry.DashboardPoseTracker;
// import com.pedropathing.telemetry.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.hardware.AutoShootingController;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import com.pedropathing.follower.Follower;

/**
 * Test OpMode for the new AutoShootingController
 * Uses FullTesting's shooting logic in a separate class
 */
@Disabled
@TeleOp(name = "Auto Shooter Test")
public class AutoShooterTest extends OpMode {
    
    private Follower follower;  // Follower includes Pinpoint localization (PedroPathing 2.0)
    private Telemetry telemetryA;
    
    private DcMotorEx fl, fr, bl, br;
    private AutoShootingController shootingController;
    
    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(25);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize Follower with Pinpoint localizer (PedroPathing 2.0)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
        
        // Initialize drive motors
        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");
        
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        // Initialize shooting controller (FullTesting logic)
        shootingController = new AutoShootingController(hardwareMap);
        
        telemetryA.addLine("Auto Shooter Test Initialized");
        telemetryA.addLine("===============================");
        telemetryA.addLine("Uses FullTesting shooting logic");
        telemetryA.addLine("");
        telemetryA.addLine("Controls:");
        telemetryA.addLine("  Right Trigger = Run shooter");
        telemetryA.addLine("  Right Stick = Drive");
        telemetryA.addLine("  Left Stick X = Rotate");
        telemetryA.update();
    }
    
    @Override
    public void loop() {
        // Update Follower (which updates Pinpoint localization) - PedroPathing 2.0
        follower.update();
        
        // Get current position from Follower (Pinpoint localization)
        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();
        
        // Update dashboard
        // dashboardPoseTracker.update();
        // TODO: Drawing removed
        // Drawing.drawRobot(currentPose, "#4CAF50");
        // TODO: Drawing removed
        // Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50FF");
        // TODO: Drawing removed
        // Drawing.sendPacket();
        
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
        
        // ==================== SHOOTER CONTROL ====================
        boolean shooterOn = gamepad1.right_trigger > 0.1;
        
        // Update shooter using FullTesting logic
        AutoShootingController.ShooterStatus status = 
            shootingController.update(shooterOn, currentX, currentY);
        
        // ==================== TELEMETRY ====================
        telemetryA.addLine("=== AUTO SHOOTER TEST ===");
        telemetryA.addData("Position", String.format("(%.1f, %.1f) @ %.1f°", 
            currentX, currentY, Math.toDegrees(currentHeading)));
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== SHOOTER STATUS ===");
        telemetryA.addData("Shooter", shooterOn ? "RUNNING" : "STOPPED");
        telemetryA.addData("At Target Speed", status.atTargetSpeed ? "✓ YES" : "NO");
        telemetryA.addData("Distance Range", status.isLongRange ? "LONG (≥6ft)" : "SHORT (<6ft)");
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== DISTANCE ===");
        telemetryA.addData("To Goal", "%.1f inches (%.1f feet)", status.distanceInches, status.distanceFeet);
        telemetryA.addData("Goal Zone", "(%.1f, %.1f)", 
            AutoShootingController.goalZoneX, AutoShootingController.goalZoneY);
        telemetryA.addData("", "");
        
        telemetryA.addLine("=== VELOCITY ===");
        telemetryA.addData("Target RPM", "%.0f", status.targetRPM);
        telemetryA.addData("Current RPM", "%.0f", status.avgRPM);
        telemetryA.addData("Right Motor RPM", "%.0f", status.rightRPM);
        telemetryA.addData("Left Motor RPM", "%.0f", status.leftRPM);
        telemetryA.addData("Error RPM", "%.0f", status.getErrorRPM());
        telemetryA.addData("", "");
        
        if (shooterOn) {
            telemetryA.addLine("=== PID DEBUG ===");
            telemetryA.addData("PIDF Output", "%.4f", status.pidfOutput);
            telemetryA.addData("Additional FF", "%.4f", status.additionalFF);
            telemetryA.addData("Shooter Power", "%.3f", status.power);
            telemetryA.addData("Compensated Power", "%.3f", status.compensatedPower);
            telemetryA.addData("Active P", status.activeP);
            telemetryA.addData("Active I", status.activeI);
            telemetryA.addData("Active D", status.activeD);
            telemetryA.addData("Active F", status.activeF);
        }
        
        telemetryA.update();
    }
    
    @Override
    public void stop() {
        shootingController.stop();
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}

