package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.hardware.LimelightRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * Example OpMode using LimelightRobot - follows the BarrelRobot pattern
 */
@TeleOp(name = "Limelight Robot Test")
public class LimelightRobotTest extends OpMode {
    
    private LimelightRobot robot;
    private Telemetry telemetryA;
    
    private DcMotorEx fl, fr, bl, br;

    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.setMsTransmissionInterval(25);
        
        // Initialize PedroPathing constants
        Constants.setConstants(FConstants.class, LConstants.class);
        
        // Create LimelightRobot (similar to BarrelRobot)
        // It creates PoseUpdater internally, just like BarrelRobot creates follower
        robot = new LimelightRobot(hardwareMap, telemetryA);
        
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

        telemetryA.addLine("=================================");
        telemetryA.addLine("LIMELIGHT ROBOT TEST");
        telemetryA.addLine("=================================");
        telemetryA.addLine("");
        telemetryA.addLine("Uses BarrelRobot pattern with Limelight");
        telemetryA.addLine("Automatically relocalizes every loop!");
        telemetryA.addLine("");
        telemetryA.addLine("Controls:");
        telemetryA.addLine("  Right Stick: Drive");
        telemetryA.addLine("  Left Stick X: Rotate");
        telemetryA.update();
    }

    @Override
    public void loop() {
        // This is all you need! (Like BarrelRobot's onTick)
        robot.onTick();
        
        // Drive controls
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
        
        // Display coordinate comparison
        double pinpointX = robot.poseUpdater.getPose().getX();
        double pinpointY = robot.poseUpdater.getPose().getY();
        double pinpointHeading = robot.poseUpdater.getPose().getHeading();
        
        telemetryA.addData("", "");
        telemetryA.addLine("╔════════════════════════════════════════════╗");
        telemetryA.addLine("║    COORDINATE COMPARISON                   ║");
        telemetryA.addLine("╚════════════════════════════════════════════╝");
        telemetryA.addData("", "");
        
        // PINPOINT COORDINATES
        telemetryA.addLine("📍 PINPOINT ODOMETRY:");
        telemetryA.addData("  X", "%.2f inches", pinpointX);
        telemetryA.addData("  Y", "%.2f inches", pinpointY);
        telemetryA.addData("  Heading", "%.1f degrees", Math.toDegrees(pinpointHeading));
        telemetryA.addData("", "");
        
        // LIMELIGHT COORDINATES
        Pose limelightPose = robot.getLimelightPose();
        telemetryA.addLine("📷 LIMELIGHT (Converted to Pinpoint Format):");
        
        if (limelightPose != null) {
            double limelightX = limelightPose.getX();
            double limelightY = limelightPose.getY();
            double limelightHeading = limelightPose.getHeading();
            
            telemetryA.addData("  X", "%.2f inches", limelightX);
            telemetryA.addData("  Y", "%.2f inches", limelightY);
            telemetryA.addData("  Heading", "%.1f degrees", Math.toDegrees(limelightHeading));
            telemetryA.addData("", "");
            
            // DIFFERENCE
            double deltaX = limelightX - pinpointX;
            double deltaY = limelightY - pinpointY;
            double deltaHeading = Math.toDegrees(limelightHeading - pinpointHeading);
            double positionError = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            
            telemetryA.addLine("📊 DIFFERENCE (Limelight - Pinpoint):");
            telemetryA.addData("  Total Error", "%.2f inches", positionError);
            telemetryA.addData("  ΔX", "%+.2f inches", deltaX);
            telemetryA.addData("  ΔY", "%+.2f inches", deltaY);
            telemetryA.addData("  ΔHeading", "%+.1f degrees", deltaHeading);
            
            // Quality indicator
            String quality;
            if (positionError < 3.0) {
                quality = "✓ EXCELLENT (<3 in)";
            } else if (positionError < 6.0) {
                quality = "⚠ GOOD (3-6 in)";
            } else if (positionError < 12.0) {
                quality = "⚠ FAIR (6-12 in)";
            } else {
                quality = "❌ POOR (>12 in)";
            }
            telemetryA.addData("  Agreement", quality);
        } else {
            telemetryA.addLine("  ❌ Not Available");
            telemetryA.addLine("  No AprilTag detected or botpose unavailable");
        }
        
        telemetryA.update();
    }

    @Override
    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}

