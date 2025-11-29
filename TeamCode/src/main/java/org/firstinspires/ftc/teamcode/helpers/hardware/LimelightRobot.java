package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

/**
 * LimelightRobot - Similar to BarrelRobot but uses Limelight for relocalization
 * Handles PedroPathing follower and automatic AprilTag-based relocalization
 */
@Config
public class LimelightRobot {

    private Limelight3A limelight;
    public Follower follower; // Follower includes Pinpoint localization (PedroPathing 2.0)

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Coordinate conversion - Field dimensions
    private static final double FIELD_SIZE_INCHES = 144.0;
    private static final double CENTER_OFFSET = FIELD_SIZE_INCHES / 2.0; // 72 inches

    // Relocalization quality thresholds (based on best practices from GitHub repos)
    private static final double MAX_LATENCY_MS = 100.0; // Reject data older than 100ms
    private static final double MIN_FIELD_X = -10.0; // Reasonable bounds check (inches, in Pinpoint coords)
    private static final double MAX_FIELD_X = 154.0;
    private static final double MIN_FIELD_Y = -10.0;
    private static final double MAX_FIELD_Y = 154.0;

    public LimelightRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Initialize Follower with Pinpoint localizer (PedroPathing 2.0)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();

        this.initLimelight();
    }

    /**
     * Call this every loop - updates odometry (NO auto-relocalization)
     * PedroPathing 2.0 - uses Follower for Pinpoint localization
     */
    public void onTick() {
        // Update Follower (which updates Pinpoint localization) - PedroPathing 2.0
        follower.update();
    }

    /**
     * Get Limelight pose without relocalization - for comparison
     * Based on official FTC sample code pattern
     */
    public Pose getLimelightPose() {
        if (limelight == null) {
            return null;
        }

        // Official sample pattern: check result != null first, then isValid()
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return null;
        }

        if (!result.isValid()) {
            return null;
        }

        // Get botpose - check for null (official sample pattern)
        Pose3D botpose = result.getBotpose();
        if (botpose == null || botpose.getPosition() == null) {
            return null;
        }

        double limelightRawX_meters = botpose.getPosition().x;
        double limelightRawY_meters = botpose.getPosition().y;

        // Check if data is valid (non-zero) - Limelight returns (0,0,0) when no tags
        // visible
        if (Math.abs(limelightRawX_meters) < 0.001 && Math.abs(limelightRawY_meters) < 0.001) {
            return null;
        }

        // Convert meters to inches
        double limelightRawX = limelightRawX_meters * 39.3701;
        double limelightRawY = limelightRawY_meters * 39.3701;

        // Convert from Limelight coordinates (center-origin) to Pinpoint
        // (corner-origin)
        double pinpointX = limelightRawX + CENTER_OFFSET;
        double pinpointY = limelightRawY + CENTER_OFFSET;

        // Get heading and normalize to 0-2π range
        double headingDegrees = botpose.getOrientation().getYaw();
        double pinpointHeading = Math.toRadians(headingDegrees);
        // Normalize heading to 0 to 2π
        while (pinpointHeading < 0) {
            pinpointHeading += 2 * Math.PI;
        }
        while (pinpointHeading >= 2 * Math.PI) {
            pinpointHeading -= 2 * Math.PI;
        }

        return new Pose(pinpointX, pinpointY, pinpointHeading);
    }

    /**
     * Initialize Limelight - Based on official FTC sample code
     */
    private void initLimelight() {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            // Set pipeline before starting (official sample pattern)
            limelight.pipelineSwitch(0); // Use pipeline 0 (AprilTag)

            // Start polling for data - MUST call start() or getLatestResult() returns null
            limelight.start();

            // Verify connection by checking status
            LLStatus status = limelight.getStatus();
            if (status != null) {
                telemetry.addLine("✓ Limelight initialized");
                telemetry.addData("  Name", status.getName());
                telemetry.addData("  Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());
            } else {
                telemetry.addLine("⚠ Limelight started but status unavailable");
            }
        } catch (Exception e) {
            telemetry.addLine("❌ Limelight initialization failed: " + e.getMessage());
            limelight = null;
        }
    }

    /**
     * Relocalize from AprilTag - Based on official FTC sample code and best
     * practices
     * Uses Limelight's MegaTag/botpose for field-relative robot position
     * Includes quality checks: latency, bounds validation, and data freshness
     */
    public void relocaliseFromAprilTag() {
        if (limelight == null) {
            telemetry.addData("AprilTag", "Limelight not initialized");
            return;
        }

        // Official sample pattern: check result != null first
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            telemetry.addData("AprilTag", "No data available");
            return;
        }

        // Then check if result is valid
        if (!result.isValid()) {
            telemetry.addData("AprilTag", "Invalid result");
            return;
        }

        // Check latency - reject stale data (best practice from GitHub repos)
        double captureLatency = result.getCaptureLatency();
        double targetingLatency = result.getTargetingLatency();
        double totalLatency = captureLatency + targetingLatency;

        if (totalLatency > MAX_LATENCY_MS) {
            telemetry.addData("AprilTag", "Data too stale (%.1fms)", totalLatency);
            return;
        }

        // Get botpose from Limelight (MegaTag localization)
        Pose3D botpose = result.getBotpose();
        if (botpose == null || botpose.getPosition() == null) {
            telemetry.addData("AprilTag", "No botpose data");
            return;
        }

        // Extract raw Limelight coordinates (meters, center-origin)
        double limelightRawX_meters = botpose.getPosition().x;
        double limelightRawY_meters = botpose.getPosition().y;

        // Check if data is valid (non-zero) - Limelight returns (0,0,0) when no tags
        // visible
        if (Math.abs(limelightRawX_meters) < 0.001 && Math.abs(limelightRawY_meters) < 0.001) {
            telemetry.addData("AprilTag", "No tags visible");
            return;
        }

        // Convert meters to inches
        double limelightRawX = limelightRawX_meters * 39.3701;
        double limelightRawY = limelightRawY_meters * 39.3701;

        // Convert from Limelight coordinates (center-origin) to Pinpoint
        // (corner-origin)
        // Limelight: (0,0) = field center
        // Pinpoint: (0,0) = bottom-left corner
        // Field is 144" × 144", so center is at (72, 72) in Pinpoint coords
        // Conversion: pinpoint = limelight + 72
        double pinpointX = limelightRawX + CENTER_OFFSET;
        double pinpointY = limelightRawY + CENTER_OFFSET;

        // Bounds check - reject unreasonable poses (best practice from GitHub repos)
        if (pinpointX < MIN_FIELD_X || pinpointX > MAX_FIELD_X ||
                pinpointY < MIN_FIELD_Y || pinpointY > MAX_FIELD_Y) {
            telemetry.addData("AprilTag", "Pose out of bounds (%.1f, %.1f)", pinpointX, pinpointY);
            return;
        }

        // Get heading in degrees, convert to radians
        double headingDegrees = botpose.getOrientation().getYaw();
        double pinpointHeading = Math.toRadians(headingDegrees);

        // Normalize heading to 0 to 2π range
        while (pinpointHeading < 0) {
            pinpointHeading += 2 * Math.PI;
        }
        while (pinpointHeading >= 2 * Math.PI) {
            pinpointHeading -= 2 * Math.PI;
        }

        // Set pose using Follower (PedroPathing 2.0)
        try {
            follower.setPose(new Pose(pinpointX, pinpointY, pinpointHeading));
            telemetry.addData("AprilTag", "✓ Relocalized");
            telemetry.addData("  Latency", "%.1fms", totalLatency);
            telemetry.addData("  Pose", "X: %.2f, Y: %.2f, H: %.1f°",
                    pinpointX, pinpointY, Math.toDegrees(pinpointHeading));
        } catch (Exception e) {
            telemetry.addData("Relocalize Error", e.getMessage());
        }
    }

    // Distance calculation constants (match AprilTagDistanceTest)
    public static double LIMELIGHT_HEIGHT_INCHES = 13.5; // Height of Limelight lens
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 19.0; // Upward tilt angle (User)
    public static double PITCH_OFFSET_DEG = 19.0; // Compensation
    public static boolean INVERT_TY = false; // Disabled based on data

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

        // Get primary target (fiducial)
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty())
            return -1;

        // Use the first detected tag (or add logic to filter by ID)
        LLResultTypes.FiducialResult target = fiducials.get(0);

        double ty = target.getTargetYDegrees();
        if (INVERT_TY)
            ty = -ty;
        double heightDifference = targetHeightInches - LIMELIGHT_HEIGHT_INCHES;
        double angleToTarget = LIMELIGHT_MOUNT_ANGLE_DEG + PITCH_OFFSET_DEG + ty;

        // Avoid division by zero or negative distances from bad angles
        if (Math.abs(angleToTarget) < 0.1)
            return -1;

        double distance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
        return distance;
    }

    /**
     * Get the horizontal offset (tx) to the primary target
     * 
     * @return tx in degrees, or 0 if no target
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
     * 
     * @return ty in degrees, or 0 if no target
     */
    public double getTy() {
        if (limelight == null)
            return 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            return INVERT_TY ? -ty : ty;
        }
        return 0;
    }

    /**
     * Calculate steering correction to aim at the target
     * 
     * @param kP       Proportional gain (e.g., 0.02 to start)
     * @param maxPower Maximum turn power to apply (e.g., 0.5)
     * @return Turn power (-1.0 to 1.0), positive = turn right (usually)
     */
    public double getSteeringCorrection(double kP, double maxPower) {
        double tx = getTx();

        // Simple Proportional Controller
        // If tx is positive (target is to the right), we need to turn right (positive
        // power)
        // Check your robot's rotation direction!
        double steering = tx * kP;

        // Clamp to max power
        if (steering > maxPower)
            steering = maxPower;
        if (steering < -maxPower)
            steering = -maxPower;

        return steering;
    }

    /**
     * This function adds a bezier line of the robot's historical positions. Call
     * this each frame
     */
    // TODO: Drawing and PoseUpdater removed in PedroPathing 2.0
    private void drawCurrentAndHistory() {
        // Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        // drawCurrent();
    }

    /**
     * This function updates the Robot's position on the field to what the
     * PoseUpdater thinks it's position is.
     */
    private void drawCurrent() {
        // TODO: Drawing and PoseUpdater removed in PedroPathing 2.0
        // try {
        // Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        // Drawing.sendPacket();
        // } catch (Exception e) {
        // throw new RuntimeException("Drawing failed " + e);
        // }
    }
}
