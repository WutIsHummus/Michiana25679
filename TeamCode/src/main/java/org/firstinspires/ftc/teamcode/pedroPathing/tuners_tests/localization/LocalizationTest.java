package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.localization;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;

import java.util.Arrays;
import java.util.List;

//import pedropathing.constants.*;

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot
 * on FTC Dashboard (192/168/43/1:8080/dash). You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config
@TeleOp(group = "A", name = "A Localization Test")
public class LocalizationTest extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private Servo turret1;
    private Servo turret2;

    private Limelight3A limelight;

    // Target coordinates for distance calculation
    public static double targetX = 128.0;
    public static double targetY = 128.0;
    
    // Height measurements
    public static double aprilTagHeight = 30.0; // AprilTag height in inches
    public static double limelightHeight = 13.5; // Limelight height in inches
    public static double heightDifference = aprilTagHeight - limelightHeight; // 16.3 inches
    public static double limelightMountAngle = 19.0; // Limelight mount angle in degrees
    
    // Turret servo constants
    public static double turretCenterPosition = 0.51; // Servo position for 0 degrees
    public static double turretLeftPosition = 0.275; // Servo position for max left
    public static double turretRightPosition = 0.745; // Servo position for max right
    public static double turretMaxAngle = 90.0; // Max angle in degrees (left or right from center)

    /**
     * This initializes the PoseUpdater, the mecanum drive motors, and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        
        // Initialize turret servos
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.setMsTransmissionInterval(11);
        
        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
            telemetryA.addLine("Limelight initialized successfully");
        } catch (Exception e) {
            telemetryA.addLine("Warning: Limelight not found - " + e.getMessage());
            limelight = null;
        }
        
        telemetryA.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the FTC
     * Dashboard telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        poseUpdater.update();
        dashboardPoseTracker.update();

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // this is strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        // Get current position
        double currentX = poseUpdater.getPose().getX();
        double currentY = poseUpdater.getPose().getY();

        // Calculate distance to target (119, 119)
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // Calculate angle to target relative to field (0 degrees = east, 90 = north)
        double angleToTargetField = Math.atan2(deltaY, deltaX);
        
        // Calculate angle relative to robot (turret angle needed)
        // Subtract robot heading to get relative angle
        double currentHeading = poseUpdater.getPose().getHeading();
        double turretAngle = angleToTargetField - currentHeading;
        
        // Normalize angle to [-PI, PI]
        while (turretAngle > Math.PI) turretAngle -= 2 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;
        
        // Convert to degrees
        double turretAngleDegrees = Math.toDegrees(turretAngle);
        
        // Calculate servo position
        // Clamp angle to valid range
        double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDegrees));
        
        // Convert angle to servo position (FLIPPED)
        double servoPosition;
        if (clampedAngle >= 0) {
            // Positive angle = turn right
            double servoRange = turretRightPosition - turretCenterPosition; // 0.235
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            // Negative angle = turn left
            double servoRange = turretCenterPosition - turretLeftPosition; // 0.235
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }
        
        // Set servo positions
        turret1.setPosition(servoPosition);
        turret2.setPosition(servoPosition);


        telemetryA.addData("x", currentX);
        telemetryA.addData("y", currentY);
        telemetryA.addData("heading (rad)", currentHeading);
        telemetryA.addData("heading (deg)", Math.toDegrees(currentHeading));
        telemetryA.addData("", ""); // Empty line
        telemetryA.addData("Distance to Target", "%.2f inches", distance);
        telemetryA.addData("Turret Angle", "%.2f degrees", turretAngleDegrees);
        telemetryA.addData("Turret Servo Position", "%.3f", servoPosition);
        if (turretAngleDegrees < -turretMaxAngle || turretAngleDegrees > turretMaxAngle) {
            telemetryA.addData("WARNING", "Target out of turret range!");
        }
        
        // Add Limelight data
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetryA.addData("", ""); // Empty line
                telemetryA.addLine("=== Limelight Data ===");
                telemetryA.addData("tx (degrees)", "%.2f", result.getTx());
                telemetryA.addData("ty (degrees)", "%.2f", result.getTy());
                
                // Display AprilTag fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (!fiducialResults.isEmpty()) {
                    telemetryA.addData("AprilTags Detected", fiducialResults.size());
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        double tx = fr.getTargetXDegrees();
                        double ty = fr.getTargetYDegrees();
                        
                        telemetryA.addData("  Tag ID", "%d", fr.getFiducialId());
                        telemetryA.addData("    X (degrees)", "%.2f", tx);
                        telemetryA.addData("    Y (degrees)", "%.2f", ty);
                        
                        // Calculate distance using height difference, mount angle, and vertical angle
                        double angleToTarget = limelightMountAngle + ty;
                        
                        if (Math.abs(angleToTarget) > 0.5) { // Only calculate if we have a valid angle
                            // Horizontal distance calculation accounting for mount angle
                            double horizontalDistance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
                            
                            // Diagonal distance from lens to center of AprilTag
                            double diagonalDistance = Math.sqrt(
                                horizontalDistance * horizontalDistance + 
                                heightDifference * heightDifference
                            );
                            
                            // Convert horizontal angle to inches offset
                            double x_inches = horizontalDistance * Math.tan(Math.toRadians(tx));
                            
                            telemetryA.addData("    Angle to Target", "%.2f deg", angleToTarget);
                            telemetryA.addData("    Horizontal Dist", "%.2f inches", horizontalDistance);
                            telemetryA.addData("    Diagonal Dist", "%.2f inches", diagonalDistance);
                            telemetryA.addData("    X Offset", "%.2f inches", x_inches);
                            telemetryA.addData("    Height Diff", "%.2f inches", heightDifference);
                        } else {
                            telemetryA.addData("    Distance", "Invalid angle");
                        }
                        
                        // Also show the 3D pose data if available
                        org.firstinspires.ftc.robotcore.external.navigation.Pose3D targetPose = fr.getRobotPoseTargetSpace();
                        if (targetPose != null && targetPose.getPosition() != null) {
                            telemetryA.addData("    Pose X", "%.2f", targetPose.getPosition().x);
                            telemetryA.addData("    Pose Y", "%.2f", targetPose.getPosition().y);
                            telemetryA.addData("    Pose Z", "%.2f", targetPose.getPosition().z);
                        }
                    }
                } else {
                    telemetryA.addData("AprilTags", "None detected");
                }
            } else {
                telemetryA.addData("", ""); // Empty line
                telemetryA.addData("Limelight", "No valid data");
            }
        }
        
        telemetryA.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}
