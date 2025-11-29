package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

/**
 * Abstract base class for OpModes that use Limelight.
 * Provides distance calculation and steering correction methods.
 */
public abstract class LimelightOpMode extends OpMode {

    protected Limelight3A limelight;

    // Distance calculation constants
    public static double LIMELIGHT_HEIGHT_INCHES = 13.5; // Height of Limelight lens
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 19.0; // User confirmed physical angle
    public static double PITCH_OFFSET_DEG = 19.0; // Compensation for unexplained sensor/mount discrepancy

    // Camera Position Offsets (relative to robot center)
    public static double CAMERA_OFFSET_FORWARD = 5.5; // Inches forward of center
    public static double CAMERA_OFFSET_RIGHT = -0.5; // Inches right of center (negative = left)

    // Inversion Flags (Use if camera is mounted upside down or settings are wrong)
    public static boolean INVERT_TX = false;
    public static boolean INVERT_TY = false; // Raw values are correct for high mount angle

    /**
     * Initialize the Limelight. Call this in your init() method.
     */
    public void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Default to pipeline 0
        limelight.start();
    }

    /**
     * Calculate horizontal distance to a target using trigonometry
     * d = (h2 - h1) / tan(a1 + a2)
     * 
     * @param targetHeightInches Height of the target (AprilTag) center in inches
     * @return Distance in inches, or -1 if no target found/invalid
     */
    public double getDistanceToTag(double targetHeightInches) {
        if (limelight == null)
            return -1;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid())
            return -1;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty())
            return -1;

        // Use the first detected tag
        LLResultTypes.FiducialResult target = fiducials.get(0);

        double ty = target.getTargetYDegrees();
        if (INVERT_TY) {
            ty = -ty;
        }

        double heightDifference = targetHeightInches - LIMELIGHT_HEIGHT_INCHES;
        // Formula: tan(Mount + Offset + ty)
        // We use 19.0 (Mount) + 19.0 (Offset) = 38.0 Effective Angle
        double angleToTarget = LIMELIGHT_MOUNT_ANGLE_DEG + PITCH_OFFSET_DEG + ty;

        // Safety check: if angle is too close to 0, distance calculation explodes
        if (Math.abs(angleToTarget) < 0.5)
            return -1;

        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }

    /**
     * Get the horizontal offset (tx) to the primary target
     */
    public double getTx() {
        if (limelight == null)
            return 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }

    /**
     * Get the vertical offset (ty) to the primary target
     */
    public double getTy() {
        if (limelight == null)
            return 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0;
    }

    protected com.pedropathing.follower.Follower follower;

    /**
     * Initialize the Follower (Pinpoint) and Limelight.
     */
    public void initRobot() {
        // Initialize Follower
        follower = org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants.createFollower(hardwareMap);
        follower.setStartingPose(new com.pedropathing.geometry.Pose(0, 0, 0)); // Default, user should override

        // Initialize Limelight
        initLimelight();
    }

    /**
     * Calculate the robot's pose based on Limelight distance and AprilTag location.
     * Does NOT update the Follower's pose.
     * 
     * @param tagX      The field X coordinate of the tag (inches).
     * @param tagY      The field Y coordinate of the tag (inches).
     * @param tagHeight The height of the tag (inches).
     * @return The calculated Pose (X, Y, Heading from Pinpoint), or null if
     *         calculation failed.
     */
    public com.pedropathing.geometry.Pose calculatePoseFromTag(double tagX, double tagY, double tagHeight) {
        if (limelight == null || follower == null)
            return null;

        double distance = getDistanceToTag(tagHeight);
        if (distance == -1)
            return null;

        double tx = getTx(); // Bearing to tag relative to robot front

        // Get Current Heading from Pinpoint
        double robotHeading = follower.getPose().getHeading();

        // Calculate Field Position of the CAMERA
        // Angle from Camera to Tag (Field Relative)
        // Robot Heading - tx (assuming tx is positive right)
        double absAngleToTag = robotHeading - Math.toRadians(tx);

        // Camera X/Y
        double cameraX = tagX - distance * Math.cos(absAngleToTag);
        double cameraY = tagY - distance * Math.sin(absAngleToTag);

        // Calculate Robot Center from Camera Position
        // Robot = Camera - OffsetRotated
        // Offset Vector: (Forward, Right)
        // Rotated:
        // X_off = F * cos(H) - R * sin(H)
        // Y_off = F * sin(H) + R * cos(H)

        double offsetX = CAMERA_OFFSET_FORWARD * Math.cos(robotHeading) - CAMERA_OFFSET_RIGHT * Math.sin(robotHeading);
        double offsetY = CAMERA_OFFSET_FORWARD * Math.sin(robotHeading) + CAMERA_OFFSET_RIGHT * Math.cos(robotHeading);

        double robotX = cameraX - offsetX;
        double robotY = cameraY - offsetY;

        return new com.pedropathing.geometry.Pose(robotX, robotY, robotHeading);
    }

    /**
     * Update the robot's pose using Limelight distance and AprilTag location.
     * 
     * @param tagId      The ID of the tag to use.
     * @param tagX       The field X coordinate of the tag (inches).
     * @param tagY       The field Y coordinate of the tag (inches).
     * @param tagHeading The field heading of the tag (radians) - usually 0, 90,
     *                   180, etc.
     * @return true if pose was updated, false otherwise.
     */
    public boolean updatePoseWithLimelight(int tagId, double tagX, double tagY, double tagHeading) {
        return updatePoseWithLimelight(tagId, tagX, tagY, tagHeading, 30.0);
    }

    public boolean updatePoseWithLimelight(int tagId, double tagX, double tagY, double tagHeading, double tagHeight) {
        com.pedropathing.geometry.Pose calculatedPose = calculatePoseFromTag(tagX, tagY, tagHeight);

        if (calculatedPose != null) {
            follower.setPose(calculatedPose);
            return true;
        }
        return false;
    }

    /**
     * Calculate steering correction to aim at the target
     * 
     * @param kP       Proportional gain
     * @param maxPower Maximum turn power
     * @return Turn power (-1.0 to 1.0)
     */
    public double getSteeringCorrection(double kP, double maxPower) {
        double tx = getTx();
        double steering = tx * kP;

        if (steering > maxPower)
            steering = maxPower;
        if (steering < -maxPower)
            steering = -maxPower;

        return steering;
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
