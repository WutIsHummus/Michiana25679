package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.LimelightOpMode;
import org.firstinspires.ftc.teamcode.helpers.hardware.LimelightRobot;

@Config
@TeleOp(name = "Limelight Distance Check", group = "Test")
public class LimelightDistanceCheck extends LimelightOpMode {

    // Configurable via Dashboard
    public static double TARGET_HEIGHT_INCHES = 29.0; // Confirmed by user
    public static double ALIGNMENT_KP = 0.02; // Proportional gain for steering
    public static double MAX_TURN_POWER = 0.5; // Max turn power

    // Test Tag Coordinates (Change on Dashboard to match your test setup)
    public static double TEST_TAG_X = 72.0;
    public static double TEST_TAG_Y = 72.0;
    public static double TEST_TAG_HEADING = 0.0; // Radians (0.0 based on reference repo)

    // Camera Tuning Variables (Edit in Dashboard to fix distance/pose)
    public static double TEST_CAMERA_HEIGHT = 13.5;
    public static double TEST_MOUNT_ANGLE = 19.0;
    public static double TEST_PITCH_OFFSET = 19.0; // Compensation
    public static double TEST_OFFSET_FORWARD = 5.5;
    public static double TEST_OFFSET_RIGHT = -0.5;

    // Inversion Toggles (Check these if camera is upside down)
    public static boolean TEST_INVERT_TX = false;
    public static boolean TEST_INVERT_TY = false; // Default to false (Standard)

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Robot (Pinpoint + Limelight)
        initRobot();

        telemetry.addLine("Initialized LimelightDistanceCheck");
        telemetry.addLine("Use Dashboard to adjust TARGET_HEIGHT_INCHES and ALIGNMENT_KP");
        telemetry.addLine("TUNE Camera Height/Angle if Distance is wrong!");
        telemetry.addLine("If Distance is HUGE (~30ft) but robot is close -> Try INVERT_TY!");
        telemetry.addLine("Press A to update Pinpoint Pose from Tag");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Sync Dashboard Tuning Variables to Base Class Statics
        LimelightOpMode.LIMELIGHT_HEIGHT_INCHES = TEST_CAMERA_HEIGHT;
        LimelightOpMode.LIMELIGHT_MOUNT_ANGLE_DEG = TEST_MOUNT_ANGLE;
        LimelightOpMode.PITCH_OFFSET_DEG = TEST_PITCH_OFFSET;
        LimelightOpMode.CAMERA_OFFSET_FORWARD = TEST_OFFSET_FORWARD;
        LimelightOpMode.CAMERA_OFFSET_RIGHT = TEST_OFFSET_RIGHT;
        LimelightOpMode.INVERT_TX = TEST_INVERT_TX;
        LimelightOpMode.INVERT_TY = TEST_INVERT_TY; // Use Dashboard value

        // Update Follower (Pinpoint)
        follower.update();

        // Get distance using base class method
        double distance = getDistanceToTag(TARGET_HEIGHT_INCHES);
        double tx = getTx();
        double ty = getTy();

        // Calculate steering correction
        double steering = getSteeringCorrection(ALIGNMENT_KP, MAX_TURN_POWER);

        // Display Current Pinpoint Pose
        com.pedropathing.geometry.Pose pinpointPose = follower.getPose();
        telemetry.addData("Pinpoint Pose", "X: %.2f, Y: %.2f, H: %.2f°",
                pinpointPose.getX(), pinpointPose.getY(), Math.toDegrees(pinpointPose.getHeading()));

        // Calculate Limelight Pose (without updating Pinpoint)
        com.pedropathing.geometry.Pose limelightPose = calculatePoseFromTag(TEST_TAG_X, TEST_TAG_Y,
                TARGET_HEIGHT_INCHES);

        if (limelightPose != null) {
            telemetry.addData("Limelight Pose", "X: %.2f, Y: %.2f, H: %.2f°",
                    limelightPose.getX(), limelightPose.getY(), Math.toDegrees(limelightPose.getHeading()));

            // Calculate Error
            double xError = Math.abs(pinpointPose.getX() - limelightPose.getX());
            double yError = Math.abs(pinpointPose.getY() - limelightPose.getY());
            double distError = Math.sqrt(xError * xError + yError * yError);

            telemetry.addData("Error", "Dist: %.2f (X: %.2f, Y: %.2f)", distError, xError, yError);

            // Update Pose on Button Press (Optional)
            if (gamepad1.a) {
                follower.setPose(limelightPose);
                telemetry.addData("Pose Update", "SYNCED");
            }
        } else {
            telemetry.addData("Limelight Pose", "Calculating... (No Tag)");
        }

        telemetry.addLine("-----------------");

        if (distance != -1) {
            telemetry.addData("Distance (Inches)", "%.2f", distance);
            telemetry.addData("tx", "%.2f", tx);
            telemetry.addData("ty", "%.2f", ty);
            telemetry.addData("Steering Correction", "%.3f", steering);
        }

        telemetry.update();
    }
}
