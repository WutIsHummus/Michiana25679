package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
// TODO: PoseUpdater removed in PedroPathing 2.0
// import com.pedropathing.telemetry.PoseUpdater;
// import com.pedropathing.telemetry.DashboardPoseTracker;
// import com.pedropathing.telemetry.Drawing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import java.util.List;

//import pedropathing.constants.*;

/**
 * Cosmobots Blue Teleop with Limelight Relocalization
 * This is a duplicate of CosmobotsBlueTeleop with added Limelight relocalization features:
 * - Automatic periodic relocalization (every 2 seconds when tags visible)
 * - Manual relocalization (D-pad Up)
 * - Limelight-based distance calculations for more accurate shooting
 * 
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 2.0 - Added Limelight relocalization
 */
@Config
@TeleOp(name = "2 - Cosmobots - Blue (Limelight)")
public class CosmobotsBlueTeleopLimelight extends OpMode {
    private Follower follower;  // Follower includes Pinpoint localization
    private Telemetry telemetryA;

    private DcMotorEx fl, fr, bl, br;

    private Servo turret1;
    private Servo turret2;

    private Limelight3A limelight;

    // Shooter hardware
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1;
    private PIDFController shooterPID;

    // Target coordinates for turret aiming
    // === Blue-Side Mirrored Field Constants ===

    // Aim target for turret (mirrored X)
    public static double targetX = 144.0 - 128.0; // mirror of Red-side 128.0 → 16.0
    public static double targetY = 125.0;
    // Low-power shooter mode (Right Trigger)
    public static double LOW_POWER_RPM = 800.0;
    public static double LOW_POWER_TRIGGER_THRESHOLD = 0.1;  // how hard you have to press RT


    // Goal zone coordinates (mirrored X)
    public static double goalZoneX = 144.0 - 116.0; // mirror of Red-side 116.0 → 28.0
    public static double goalZoneY = 116.0;

    // Snap-to-pose (mirrored X and heading)
    public static double SNAP_X = 144.0 - 101.3293; // mirror of Red-side snap point → 42.6707
    public static double SNAP_Y = 123.7003;
    public static double SNAP_HEADING_DEG = (180.0 - 359.0 + 360.0) % 360.0; // mirrors 359° → 181°

    public static double turretTrimDeg = 0.0; // stays the same
    public static double TRIM_STEP_DEG = 3.0;


    // Height measurements
    public static double aprilTagHeight = 30.0; // AprilTag height in inches
    public static double limelightHeight = 13.5; // Limelight height in inches
    public static double heightDifference = aprilTagHeight - limelightHeight; // 16.3 inches
    
    // Camera mount angles (in degrees)
    // Pitch = tilt up/down (positive = tilted up)
    // Yaw = rotation left/right (positive = rotated right/clockwise when viewed from above)
    // Roll = tilt left/right (usually 0, positive = tilted right)
    public static double CAMERA_PITCH_DEG = 19.0;  // Camera tilt angle (pitch) - how much camera is tilted up
    public static double CAMERA_YAW_DEG = 0.0;     // Camera yaw offset - if camera points left/right (0 = straight forward)
    public static double CAMERA_ROLL_DEG = 0.0;     // Camera roll - if camera is tilted sideways (usually 0)
    
    // Legacy constant for backward compatibility
    public static double limelightMountAngle = CAMERA_PITCH_DEG;
    
    // Limelight camera offset from robot center (in robot-relative coordinates)
    // These should match what's configured in Limelight Settings -> Robot
    // Positive forward = camera is forward of robot center
    // Positive right = camera is to the right of robot center
    public static double CAMERA_FORWARD_OFFSET_INCHES = 5.5;   // Camera is 5.5 inches forward of robot center
    public static double CAMERA_RIGHT_OFFSET_INCHES = -0.5;   // Camera is 0.5 inches to the LEFT (negative = left)
    public static double CAMERA_UP_OFFSET_INCHES = 13.5;      // Camera height (already used in limelightHeight)

    // Turret servo constants
    public static double turretCenterPosition = 0.51; // Servo position for 0 degrees
    public static double turretLeftPosition = 0.15; // Servo position for max left
    public static double turretRightPosition = 0.85; // Servo position for max right
    public static double turretMaxAngle = 140; // Max angle in degrees (left or right from center)

    // Shooter PIDF Constants - From VelocityFinder
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;

    // Short range PIDF (< 6 feet)
    public static double p = 0.0015;
    public static double i = 0.0;
    public static double d = 0.0000;
    public static double f = 0.0009;
    public static double kV = 0.0008;
    public static double kS = 0.01;
    public static double I_ZONE = 250.0;
    public static double hood1Position = 0.54;

    // Long range PIDF (>= 6 feet)
    public static double pLong = 0.008;
    public static double iLong = 0.0;
    public static double dLong = 0;
    public static double fLong = 0.00089;
    public static double kVLong = 0;
    public static double kSLong = 0.01;
    public static double I_ZONE_LONG = 250.0;
    public static double hood1PositionLong = 0.45;

    private boolean lastB = false;


    // Linear regression for RPM calculation: RPM = 100 * (feet_from_goal) + 1150
    // where x is feet from goal zone, y is RPM
    // Formula: y = 100x + 1150
    private static final double RPM_SLOPE = 80.0;  // m in y = mx + b
    private static final double RPM_INTERCEPT = 1050.0;  // b in y = mx + b (was 1150)
    // b in y = mx + b

    // Far shooting RPM cap (for distances >= 7 feet)
    public static double FAR_SHOOTING_RPM_MAX = 1950.0;  // Reduced from 2100

    // Auto-shoot state machine
    private boolean lastA = false;
    private boolean lastLeftTrigger = false;
    private boolean shooting = false;
    private boolean autoTransfer = false;
    private int shootState = 0;
    private int transferState = 0;
    private ElapsedTime shootTimer;
    private ElapsedTime transferTimer;

    // RPM tolerance for "at target" detection
    public static double RPM_TOLERANCE = 100.0;
    private boolean shootingconstant = true;
    private boolean lastY = false;

    // --- Hood regression (distance-based) ---
    public static double HOOD_MIN_POS = 0.45;      // flattest shot (far)
    public static double HOOD_MAX_POS = 0.54;      // highest arc (close)
    public static double HOOD_MIN_DIST_FT = 0.5;   // start of interpolation range
    public static double HOOD_MAX_DIST_FT = 7.0;   // end of interpolation range

    // --- Turret backlash compensation for turret1 only ---
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    // Voltage compensation (always on)
    private static final double NOMINAL_VOLTAGE = 12.0;

    // --- add near other fields ---
    private boolean lastX = false;        // edge detector for pose snap
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;   // edge detector for manual relocalization
    // LED strips (GoBILDA PWM lights)
    private Servo led1;
    private Servo led2;
    
    // Limelight relocalization
    private ElapsedTime relocalizeTimer;
    private static final double AUTO_RELOCALIZE_INTERVAL_SEC = 2.0;  // Relocalize every 2 seconds when tags visible
    private static final double MAX_LATENCY_MS = 100.0;  // Reject stale Limelight data
    private static final double MAX_POSE_CHANGE_INCHES = 12.0;  // Reject relocalization if pose changes more than 12 inches
    private static final double MAX_HEADING_CHANGE_RAD = Math.toRadians(30.0);  // Reject if heading changes more than 30 degrees
    
    // AprilTag field positions in LIMELIGHT coordinates (center of field = 0,0)
    // Field is 144" x 144", so center is at (0,0) in Limelight coords
    // These are the known positions of AprilTags on the field
    // TODO: Update these with actual AprilTag positions for your field
    private static final double[][] APRILTAG_POSITIONS = {
        // {tagId, x_inches_from_center, y_inches_from_center}
        {11, -60.0, 72.0},   // Example: Top-left structure tag
        {12, 60.0, 72.0},    // Example: Top-right structure tag
        {13, -60.0, -72.0},  // Example: Bottom-left structure tag
        {14, 60.0, -72.0},   // Example: Bottom-right structure tag
        // Add more tags as needed
    };

    // LED color positions (from GoBILDA chart)
    public static double LED_OFF    = 0.0;
    public static double LED_RED    = 0.277;
    public static double LED_YELLOW = 0.388;
    public static double LED_GREEN  = 0.500;
    public static double LED_BLUE   = 0.611;



    /**
     * This initializes the Follower (with Pinpoint localization), the mecanum drive motors, and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        // Initialize Follower with Pinpoint localizer (configured in Constants.java)
        follower = Constants.createFollower(hardwareMap);
        
        // Try to restore saved pose
        try {
            if (PoseStore.hasSaved()) {
                follower.setStartingPose(PoseStore.lastPose);
            } else {
                follower.setStartingPose(new Pose(0, 0, 0));
            }
        } catch (Exception ignored) {
            // Pose restore failed — use default starting pose
            follower.setStartingPose(new Pose(0, 0, 0));
        }
        
        // Start teleop drive mode
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



        // Initialize turret servos
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        // Initialize shooter hardware
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl = hardwareMap.get(DcMotorEx.class, "shootl");

        reargate = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        hood1 = hardwareMap.get(Servo.class, "hood 1");

        // Set shooter motor directions
        shootl.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeback.setDirection(DcMotorSimple.Direction.REVERSE);
        intakefront.setDirection(DcMotorSimple.Direction.REVERSE);
        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");

        // Setup all motors
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Initialize shooter PIDF controller
        shooterPID = new PIDFController(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);

        // Initialize auto-shoot timers
        shootTimer = new ElapsedTime();
        transferTimer = new ElapsedTime();
        relocalizeTimer = new ElapsedTime();

        // Set initial servo positions
        launchgate.setPosition(0.5);
        hood1.setPosition(hood1Position);
        setLedColor(LED_GREEN);
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

        telemetryA.addLine("Full Testing OpMode - Localization + Shooter + Limelight");
        telemetryA.addLine("Gamepad 1 Controls:");
        telemetryA.addLine("  - Right Stick: Mecanum drive (Y/X)");
        telemetryA.addLine("  - Left Stick X: Rotation");
        telemetryA.addLine("  - Left Bumper: Back intake");
        telemetryA.addLine("  - Right Bumper: Front intake");
        telemetryA.addLine("  - Right Trigger: Spin up shooter (RPM auto-calculated)");
        telemetryA.addLine("  - Left Trigger: Auto-transfer 3 balls when at speed");
        telemetryA.addLine("  - A Button: Auto-shoot 3 balls (distance-based)");
        telemetryA.addLine("  - D-Pad Up: Manual Limelight relocalization");
        telemetryA.addLine("  - X Button: Snap to preset pose");
        telemetryA.addLine("RPM Formula: RPM = 100 * (feet from goal) + 1150");
        telemetryA.addLine("Auto-transfer adapts to distance (fast <7ft, slow ≥7ft)");
        telemetryA.addLine("Limelight: Auto-relocalizes every 2s when tags visible");
        telemetryA.update();


        // TODO: Drawing removed
        // TODO: PoseUpdater removed
        // TODO: PoseUpdater removed
        // // // Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        // TODO: Drawing removed
        // Drawing.sendPacket();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the FTC
     * Dashboard telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {



        // --- Turret trim controls: D-pad LEFT = -3°, RIGHT = +3° (edge-triggered) ---
        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        // --- B to reset turret trim (offset) to 0 ---
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastB) {
            turretTrimDeg = 0.0;   // reset offset
        }
        lastB = bPressed;
// Low-power mode: hold RT to force shooter to 800 RPM
        boolean lowPowerMode = gamepad1.right_trigger > LOW_POWER_TRIGGER_THRESHOLD;

        if (dpadRight && !lastDpadRight) {
            turretTrimDeg += TRIM_STEP_DEG;
        }
        if (dpadLeft && !lastDpadLeft) {
            turretTrimDeg -= TRIM_STEP_DEG;
        }

        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;

        // Update Follower (which updates Pinpoint localization)
        follower.update();

        // --- snap-to-pose: gamepad1.x ---
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastX) {
            double snapHeadingRad = Math.toRadians(SNAP_HEADING_DEG);
            // Set pose using Follower
            follower.setPose(new Pose(SNAP_X, SNAP_Y, snapHeadingRad));
        }
        lastX = xPressed;
        
        // --- Manual relocalization: D-pad UP ---
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp) {
            relocalizeFromLimelight();
        }
        lastDpadUp = dpadUp;

        // Drive controls - matching VelocityFinder setup
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x * 1.1;
        double rx = gamepad1.left_stick_x;

        double scale = 1;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double flPower = (y + x + rx) / denominator * scale;
        double blPower = (y - x + rx) / denominator * scale;
        double frPower = (y - x - rx) / denominator * scale;
        double brPower = (y + x - rx) / denominator * scale;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        
        // --- Automatic periodic relocalization ---
        // Only relocalize when:
        // 1. AprilTag is visible
        // 2. NOT actively shooting
        // 3. Robot is relatively stationary
        // This prevents relocalization from interfering with turret aiming
        boolean robotMoving = Math.abs(y) > 0.1 || Math.abs(x) > 0.1 || Math.abs(rx) > 0.1;
        boolean canRelocalize = !shooting && !autoTransfer && !robotMoving;
        
        if (canRelocalize && relocalizeTimer.seconds() >= AUTO_RELOCALIZE_INTERVAL_SEC) {
            if (attemptRelocalizationFromAprilTag()) {
                relocalizeTimer.reset();  // Reset timer on successful relocalization
            } else {
                relocalizeTimer.reset();  // Reset timer even on failure to avoid blocking
            }
        }

        // Get current position from Follower (Pinpoint localization)
        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        // Calculate distance and angle to target (for turret aiming)
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Calculate angle to target relative to field (0 degrees = east, 90 = north)
        double angleToTargetField = Math.atan2(deltaY, deltaX);

        // Calculate angle relative to robot (turret angle needed)
        // Subtract robot heading to get relative angle
        double currentHeading = currentPose.getHeading();
        double turretAngle = angleToTargetField - currentHeading;

        // Normalize angle to [-PI, PI]
        while (turretAngle > Math.PI) turretAngle -= 2 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

        // Convert to degrees
        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;

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
        // Set servo positions with backlash compensation on turret1 only
        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
// clamp to [0,1] so it never explodes
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1.setPosition(turret1Pos-0.01);
        turret2.setPosition(servoPosition-0.01);


        // ===== SHOOTER CONTROL =====
        // Intake controls - split between bumpers
        // Left bumper: back intake
        if (gamepad1.left_bumper) {
            intakeback.setPower(1.0);
        } else {
            intakeback.setPower(0);
        }

        // Right bumper: front intake
        if (gamepad1.right_bumper) {
            intakefront.setPower(1.0);
        } else {
            intakefront.setPower(0);
        }


// Far vs near for auto-shoot timing
        double deltaGoalXForTiming = goalZoneX - currentX;
        double deltaGoalYForTiming = goalZoneY - currentY;
        double distanceFeetForTiming = Math.sqrt(
                deltaGoalXForTiming * deltaGoalXForTiming +
                        deltaGoalYForTiming * deltaGoalYForTiming
        ) / 12.0;

// "Far" shots: double the time between shots
        boolean isFarForShots = distanceFeetForTiming >= 6.0;
        double shotTimeScale = isFarForShots ? 16.0 : 1.0;

        // ==================== AUTO-SHOOT CONTROL ====================
        // Detect A button press for auto-shoot sequence
        boolean currentA = gamepad1.a;
        if (currentA && !lastA && !shooting) {
            // Start 3-ball shooting sequence
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }
        lastA = currentA;

        // Run shooting state machine (3 shots)
        String shootStatus = "Ready";
        if (shooting) {
            switch (shootState) {
                case 0: // Spin up shooter
                    shootStatus = "Spinning up...";
                    if (shootTimer.seconds() > 1.0) {   // keep spinup the same
                        shootState = 1;
                        shootTimer.reset();
                    }
                    break;

                case 1: // Start intakes
                    shootStatus = "Starting intakes...";
                    intakefront.setPower(-1.0);
                    intakeback.setPower(-1.0);
                    if (shootTimer.seconds() > 0.1) {   // keep intake start the same
                        shootState = 2;
                        shootTimer.reset();
                    }
                    break;

                case 2: // Fire shot 1
                    shootStatus = "Firing shot 1/3";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2 * shotTimeScale) {
                        shootState = 3;
                        shootTimer.reset();
                    }
                    break;

                case 3: // Reset gate 1
                    shootStatus = "Reset 1/3";
                    launchgate.setPosition(0.5);
                    if (shootTimer.seconds() > 0.3 * shotTimeScale) {
                        shootState = 4;
                        shootTimer.reset();
                    }
                    break;

                case 4: // Fire shot 2
                    shootStatus = "Firing shot 2/3";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2 * shotTimeScale) {
                        shootState = 5;
                        shootTimer.reset();
                    }
                    break;

                case 5: // Reset gate 2
                    shootStatus = "Reset 2/3";
                    launchgate.setPosition(0.5);
                    if (shootTimer.seconds() > 0.3 * shotTimeScale) {
                        shootState = 6;
                        shootTimer.reset();
                    }
                    break;

                case 6: // Fire shot 3
                    shootStatus = "Firing shot 3/3";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2 * shotTimeScale) {
                        shootState = 7;
                        shootTimer.reset();
                    }
                    break;

                case 7: // Reset gate 3 and stop
                    shootStatus = "Complete!";
                    launchgate.setPosition(0.5);
                    intakefront.setPower(0);
                    intakeback.setPower(0);
                    if (shootTimer.seconds() > 0.2 * shotTimeScale) {
                        shooting = false;
                        shootState = 0;
                    }
                    break;

            }
        }

        // Shooter velocity control - controlled by shootingconstant (always true) OR auto-shoot active
        boolean yPressed = gamepad1.y;

// Toggle shootingconstant on rising edge of Y
        if (yPressed && !lastY) {
            shootingconstant = !shootingconstant;
        }
        lastY = yPressed;
        boolean shooterOn = shootingconstant || shooting;

        // Calculate distance to goal zone for RPM calculation
        // Try to use Limelight botpose for more accurate distance, fallback to odometry
        double limelightX = currentX;
        double limelightY = currentY;
        boolean usingLimelightPose = false;
        
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null && botpose.getPosition() != null) {
                    double captureLatency = result.getCaptureLatency();
                    double targetingLatency = result.getTargetingLatency();
                    double totalLatency = captureLatency + targetingLatency;
                    
                    // Only use Limelight if data is fresh and valid
                    if (totalLatency < MAX_LATENCY_MS) {
                        double rawX_meters = botpose.getPosition().x;
                        double rawY_meters = botpose.getPosition().y;
                        
                        // Check if data is valid (non-zero)
                        if (Math.abs(rawX_meters) > 0.001 || Math.abs(rawY_meters) > 0.001) {
                            // Convert from Limelight (center-origin, meters) to Pinpoint (corner-origin, inches)
                            limelightX = (rawX_meters * 39.3701) + 72.0;
                            limelightY = (rawY_meters * 39.3701) + 72.0;
                            usingLimelightPose = true;
                        }
                    }
                }
            }
        }
        
        double deltaGoalX = goalZoneX - limelightX;
        double deltaGoalY = goalZoneY - limelightY;
        double distanceToGoalInches = Math.sqrt(deltaGoalX * deltaGoalX + deltaGoalY * deltaGoalY);
        double distanceToGoalFeet = distanceToGoalInches / 12.0;  // Convert inches to feet

        // Calculate target RPM using linear regression: RPM = 100 * (feet) + 1150
        // Calculate target RPM: use formula up to 7 feet, then cap at FAR_SHOOTING_RPM_MAX
        // Calculate target RPM
        double calculatedTargetRPM;

        if (lowPowerMode) {
            // Low-power mode: fixed 800 RPM to reduce current draw
            calculatedTargetRPM = LOW_POWER_RPM;
        } else {
            // Distance-based regression with clamp
            if (distanceToGoalFeet >= 9.0) {
                calculatedTargetRPM = FAR_SHOOTING_RPM_MAX;  // far shooting RPM stays the same
            } else {
                calculatedTargetRPM = RPM_SLOPE * distanceToGoalFeet + RPM_INTERCEPT;
                calculatedTargetRPM = Math.max(1150.0, Math.min(FAR_SHOOTING_RPM_MAX, calculatedTargetRPM));
            }
        }



        // Determine if we're shooting long range (>= 6 feet)
        boolean isLongRange = distanceToGoalFeet >= 6.0;

        // Use appropriate PIDF values based on distance
        // Use appropriate PIDF values based on distance
        double currentP = isLongRange ? pLong : p;
        double currentI = isLongRange ? iLong : i;
        double currentD = isLongRange ? dLong : d;
        double currentF = isLongRange ? fLong : f;
        double currentKV = isLongRange ? kVLong : kV;
        double currentKS = isLongRange ? kSLong : kS;
        double currentIZone = isLongRange ? I_ZONE_LONG : I_ZONE;

// --- HOOD REGRESSION (distance-based between HOOD_MAX_POS and HOOD_MIN_POS) ---
        double hoodT = (distanceToGoalFeet - HOOD_MIN_DIST_FT) / (HOOD_MAX_DIST_FT - HOOD_MIN_DIST_FT);
// clamp 0–1
        hoodT = Math.max(0.0, Math.min(1.0, hoodT));

// interpolate: close (small distance) → HOOD_MAX_POS, far → HOOD_MIN_POS
        double currentHoodPos = HOOD_MAX_POS + hoodT * (HOOD_MIN_POS - HOOD_MAX_POS);

// clamp servo range just in case
        currentHoodPos = Math.max(0.0, Math.min(1.0, currentHoodPos));

// Update PIDF coefficients
        shooterPID.setPIDF(currentP, currentI, currentD, currentF);
        shooterPID.setIntegrationBounds(-currentIZone, currentIZone);

// Set hood position based on regression
        hood1.setPosition(currentHoodPos);


        // Convert target RPM to ticks per second
        double targetTPS = rpmToTicksPerSec(calculatedTargetRPM);

        // Read current velocities
        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double vAvg = 0.5 * (vR + vL);

        // Convert to RPM for display
        double shootrVelocityRPM = ticksPerSecToRPM(vR);
        double shootlVelocityRPM = ticksPerSecToRPM(vL);
        double avgVelocityRPM = ticksPerSecToRPM(vAvg);

        double shooterPower = 0;
        double pidfOutput = 0;
        double additionalFF = 0;

        if (shooterOn) {
            // PIDF control (F term is built-in and multiplied by setpoint)
            pidfOutput = shooterPID.calculate(vAvg, targetTPS);

            // Additional feedforward using kV and kS (use current values based on distance)
            double sgn = Math.signum(targetTPS);
            additionalFF = (Math.abs(targetTPS) > 1e-6) ? (currentKS * sgn + currentKV * targetTPS) : 0.0;

            // Total power (PIDF output already includes F*setpoint)
            // If using only F term, set kS and kV to 0
            shooterPower = pidfOutput + additionalFF;

            // Safety: prevent overshoot
            if (avgVelocityRPM >= calculatedTargetRPM && shooterPower > 0) {
                shooterPower = Math.min(shooterPower, 0.5);
            }

            // Clamp
            shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));

            // Voltage compensation (always on): power * (12V / currentVoltage)
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double compensatedPower = shooterPower * (NOMINAL_VOLTAGE / voltage);
            compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));

            shootr.setPower(compensatedPower);
            shootl.setPower(compensatedPower);
        } else {
            shootr.setPower(0);
            shootl.setPower(0);
            shooterPID.reset();
        }

        // ==================== LEFT TRIGGER SIMPLE SEQUENCE ====================
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.1;

        // Detect left trigger press to start sequence
        if (currentLeftTrigger && !lastLeftTrigger && !autoTransfer) {
            autoTransfer = true;
            transferState = 0;
            transferTimer.reset();
        }
        lastLeftTrigger = currentLeftTrigger;

        String transferStatus = "Ready";

        if (autoTransfer) {
            switch (transferState) {

                case 0: // FIRE 1 - open
                    transferStatus = "Fire 1 OPEN";
                    launchgate.setPosition(0.8);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.05) {
                        transferState = 1;
                        transferTimer.reset();
                    }
                    break;

                case 1: // FIRE 1 - close
                    transferStatus = "Fire 1 CLOSE";
                    launchgate.setPosition(0.5);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.1) {
                        transferState = 2;
                        transferTimer.reset();
                    }
                    break;

                case 2: // FIRE 2 - open
                    transferStatus = "Fire 2 OPEN";
                    launchgate.setPosition(0.8);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.05) {
                        transferState = 3;
                        transferTimer.reset();
                    }
                    break;

                case 3: // FIRE 2 - close
                    transferStatus = "Fire 2 CLOSE";
                    launchgate.setPosition(0.5);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.1) {
                        transferState = 4;
                        transferTimer.reset();
                    }
                    break;

                case 4: // FIRE 3 - open
                    transferStatus = "Fire 3 OPEN";
                    launchgate.setPosition(0.8);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.05) {
                        transferState = 5;
                        transferTimer.reset();
                    }
                    break;

                case 5: // FIRE 3 - close
                    transferStatus = "Fire 3 CLOSE";
                    launchgate.setPosition(0.5);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.1) {
                        transferState = 6;
                        transferTimer.reset();
                    }
                    break;

                case 6: // FIRE 4 - open
                    transferStatus = "Fire 4 OPEN";
                    launchgate.setPosition(0.8);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.05) {
                        transferState = 7;
                        transferTimer.reset();
                    }
                    break;

                case 7: // FIRE 4 - close + end
                    transferStatus = "Fire 4 CLOSE";
                    launchgate.setPosition(0.5);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.1) {
                        autoTransfer = false;
                        transferState = 0;
                    }
                    break;
            }

        } else if (!currentLeftTrigger && !autoTransfer) {
            // Manual launch gate default when not in sequence
            launchgate.setPosition(0.5);
        }
// ---------- LED STATE LOGIC ----------
// Priority:
// 1) Red  = shooter ON but NOT at target RPM
// 2) Blue = auto-shooting or auto-transfer sequence
// 3) Yellow = intaking with either bumper
// 4) Green = normal

        boolean atTargetSpeed = Math.abs(avgVelocityRPM - calculatedTargetRPM) < RPM_TOLERANCE;
        boolean intakeActive  = gamepad1.left_bumper || gamepad1.right_bumper;

        double ledColor = LED_GREEN;  // default

        if (shooterOn && !atTargetSpeed) {
            // Shooter running but not at speed yet
            ledColor = LED_RED;
        } else if (shooting || autoTransfer) {
            // Actively firing sequence
            ledColor = LED_BLUE;
        } else if (intakeActive) {
            // Just intaking
            ledColor = LED_YELLOW;
        }

        setLedColor(ledColor);
// ---------- END LED STATE LOGIC ----------


        telemetryA.addData("", ""); // Empty line
        telemetryA.addLine("=== POSITION COMPARISON ===");
        telemetryA.addLine("--- Odometry (Pinpoint) ---");
        telemetryA.addData("X", "%.2f inches", currentX);
        telemetryA.addData("Y", "%.2f inches", currentY);
        telemetryA.addData("Heading", "%.1f° (%.3f rad)", Math.toDegrees(currentHeading), currentHeading);
        
        // Calculate AprilTag position for comparison
        double aprilTagX = Double.NaN;
        double aprilTagY = Double.NaN;
        double aprilTagHeading = Double.NaN;
        int detectedTagId = -1;
        double tagDistance = 0;
        boolean aprilTagAvailable = false;
        
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (!fiducialResults.isEmpty()) {
                    LLResultTypes.FiducialResult detectedTag = fiducialResults.get(0);
                    detectedTagId = detectedTag.getFiducialId();
                    
                    // Find tag's field position
                    double tagFieldX_Limelight = 0;
                    double tagFieldY_Limelight = 0;
                    boolean tagPositionKnown = false;
                    
                    for (double[] tagPos : APRILTAG_POSITIONS) {
                        if ((int)tagPos[0] == detectedTagId) {
                            tagFieldX_Limelight = tagPos[1];
                            tagFieldY_Limelight = tagPos[2];
                            tagPositionKnown = true;
                            break;
                        }
                    }
                    
                    if (tagPositionKnown) {
                        // Calculate camera position from tag
                        double tx_raw = detectedTag.getTargetXDegrees();
                        double ty_raw = detectedTag.getTargetYDegrees();
                        
                        // Account for camera yaw offset (if camera is not pointing straight forward)
                        double tx = tx_raw - CAMERA_YAW_DEG;
                        
                        // Calculate total vertical angle to target (camera pitch + ty from Limelight)
                        double angleToTarget = CAMERA_PITCH_DEG + ty_raw;
                        
                        if (Math.abs(angleToTarget) > 0.5) {
                            // Calculate distance to tag (horizontal distance in the plane)
                            double horizontalDistance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
                            // Calculate horizontal offset (left/right) from camera center
                            double xOffset = horizontalDistance * Math.tan(Math.toRadians(tx));
                                
                                // Camera position relative to tag
                                double cameraX_relativeToTag = -xOffset;
                                double cameraY_relativeToTag = -horizontalDistance;
                                
                                // Camera absolute position
                                double cameraX_Limelight = tagFieldX_Limelight + cameraX_relativeToTag;
                                double cameraY_Limelight = tagFieldY_Limelight + cameraY_relativeToTag;
                                
                                // Account for camera offset from robot center
                                // Rotate camera offset from robot-relative to field-relative coordinates
                                double offsetX_field = CAMERA_FORWARD_OFFSET_INCHES * Math.cos(currentHeading) - 
                                                       CAMERA_RIGHT_OFFSET_INCHES * Math.sin(currentHeading);
                                double offsetY_field = CAMERA_FORWARD_OFFSET_INCHES * Math.sin(currentHeading) + 
                                                       CAMERA_RIGHT_OFFSET_INCHES * Math.cos(currentHeading);
                                
                                // Robot center position = camera position - offset
                                double robotX_Limelight = cameraX_Limelight - offsetX_field;
                                double robotY_Limelight = cameraY_Limelight - offsetY_field;
                                
                                // Convert to Pinpoint coordinates
                                aprilTagX = robotX_Limelight + 72.0;
                                aprilTagY = robotY_Limelight + 72.0;
                                aprilTagHeading = currentHeading;  // Keep current heading for now
                                tagDistance = horizontalDistance;
                                aprilTagAvailable = true;
                            }
                        }
                }
            }
        }
        
        telemetryA.addLine("--- AprilTag (Limelight) ---");
        if (aprilTagAvailable) {
            telemetryA.addData("Tag ID", "%d", detectedTagId);
            telemetryA.addData("Distance to Tag", "%.2f inches", tagDistance);
            telemetryA.addData("Camera Offset", "Fwd: %.1f\", Right: %.1f\"", 
                CAMERA_FORWARD_OFFSET_INCHES, CAMERA_RIGHT_OFFSET_INCHES);
            telemetryA.addData("X", "%.2f inches", aprilTagX);
            telemetryA.addData("Y", "%.2f inches", aprilTagY);
            telemetryA.addData("Heading", "%.1f° (using odometry)", Math.toDegrees(aprilTagHeading));
            
            // Calculate difference
            double diffX = aprilTagX - currentX;
            double diffY = aprilTagY - currentY;
            double diffDistance = Math.sqrt(diffX * diffX + diffY * diffY);
            
            telemetryA.addLine("--- Difference ---");
            telemetryA.addData("ΔX", "%.2f inches", diffX);
            telemetryA.addData("ΔY", "%.2f inches", diffY);
            telemetryA.addData("ΔDistance", "%.2f inches", diffDistance);
        } else {
            telemetryA.addData("Status", "No AprilTag visible or invalid");
        }
        
        telemetryA.addData("", ""); // Empty line
        telemetryA.addLine("=== RELOCALIZATION ===");
        telemetryA.addData("Auto-Relocalize", "Every %.1fs (when tags visible)", AUTO_RELOCALIZE_INTERVAL_SEC);
        telemetryA.addData("Last Relocalize", "%.1fs ago", relocalizeTimer.seconds());
        telemetryA.addData("Manual (D-Pad Up)", "Press to relocalize now");
        telemetryA.addData("", ""); // Empty line
        telemetryA.addLine("=== AIMING ===");
        telemetryA.addData("Turret Target", "(%.1f, %.1f)", targetX, targetY);
        telemetryA.addData("Distance to Target", "%.2f inches", distance);
        telemetryA.addData("Turret Angle", "%.2f degrees", turretAngleDegrees);
        telemetryA.addData("Turret Servo Position", "%.3f", servoPosition);
        if (turretAngleDegrees < -turretMaxAngle || turretAngleDegrees > turretMaxAngle) {
            telemetryA.addData("WARNING", "Target out of turret range!");
        }
        telemetryA.addData("Low Power Mode", lowPowerMode ? "RT: 800 RPM" : "OFF");

        // Add shooter telemetry
        telemetryA.addData("", ""); // Empty line
        telemetryA.addLine("=== Shooter Status ===");
        telemetryA.addData("Shooter", shooterOn ? "RUNNING" : "STOPPED");
        telemetryA.addData("At Target Speed", Math.abs(avgVelocityRPM - calculatedTargetRPM) < RPM_TOLERANCE ? "✓ YES" : "NO");
        if (shooting) {
            telemetryA.addData("Auto-Shoot", shootStatus);
            telemetryA.addData("State", shootState);
        }
        if (autoTransfer) {
            telemetryA.addData("Auto-Transfer", transferStatus);
            telemetryA.addData("Transfer State", transferState);
        }
        telemetryA.addData("Distance Range", isLongRange ? "LONG (≥6ft) - p=0.01, hood=0.45" : "SHORT (<6ft) - p=0.002, hood=0.54");
        telemetryA.addData("Goal Zone (RPM calc)", "(%.1f, %.1f)", goalZoneX, goalZoneY);
        telemetryA.addData("Distance to Goal", "%.2f inches (%.2f feet)", distanceToGoalInches, distanceToGoalFeet);
        telemetryA.addData("Distance Source", usingLimelightPose ? "Limelight botpose" : "Odometry");
        telemetryA.addData("Calculated Target RPM", "%.0f (RPM = 100*%.2f + 1150)", calculatedTargetRPM, distanceToGoalFeet);
        telemetryA.addData("Current RPM", "%.0f", avgVelocityRPM);
        telemetryA.addData("Right Motor RPM", "%.0f", shootrVelocityRPM);
        telemetryA.addData("Left Motor RPM", "%.0f", shootlVelocityRPM);
        telemetryA.addData("Error (RPM)", "%.0f", calculatedTargetRPM - avgVelocityRPM);

        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double compensationFactor = NOMINAL_VOLTAGE / currentVoltage;

        telemetryA.addData("Battery Voltage", "%.2f V", currentVoltage);
        telemetryA.addData("Voltage Comp", "×%.3f (12V/%.2fV)", compensationFactor, currentVoltage);
        telemetryA.addData("Shooter Power (calc)", "%.3f", shooterPower);

        if (shooterOn) {
            double compensatedPower = shooterPower * compensationFactor;
            telemetryA.addData("Actual Motor Power", "%.3f", compensatedPower);
        }

        if (shooterOn) {
            telemetryA.addData("PIDF Output", "%.4f", pidfOutput);
            telemetryA.addData("Additional FF", "%.4f (kS=%.4f + kV*TPS=%.4f)",
                    additionalFF, currentKS * Math.signum(targetTPS), currentKV * targetTPS);
            telemetryA.addData("F Term Contribution", "%.4f (F*setpoint = %.4f*%.1f)",
                    currentF * targetTPS, currentF, targetTPS);
        }
        telemetryA.addData("Front Intake", gamepad1.right_bumper ? "RUNNING" : "STOPPED");
        telemetryA.addData("Back Intake", gamepad1.left_bumper ? "RUNNING" : "STOPPED");
        telemetryA.addData("Launch Gate", gamepad1.left_trigger > 0.1 ? "FIRING" : "RESET");
        telemetryA.addData("Hood Position", "%.2f", currentHoodPos);
        telemetryA.addData("", "");
        telemetryA.addLine("=== PIDF Tuning ===");
        telemetryA.addData("P", "%.6f", p);
        telemetryA.addData("I", "%.6f", i);
        telemetryA.addData("D", "%.6f", d);
        telemetryA.addData("F", "%.6f (NEW - tune this!)", f);
        telemetryA.addData("kV", "%.6f (optional, can set to 0)", kV);
        telemetryA.addData("kS", "%.6f (optional, can set to 0)", kS);
        telemetryA.addData("Snap Button (X)", xPressed ? "pressed" : "idle");

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

                        // Account for camera yaw offset (if camera is not pointing straight forward)
                        double tx_adjusted = tx - CAMERA_YAW_DEG;
                        
                        // Calculate distance using height difference, mount angle, and vertical angle
                        // Total vertical angle = camera pitch + ty from Limelight
                        double angleToTarget = CAMERA_PITCH_DEG + ty;

                        if (Math.abs(angleToTarget) > 0.5) { // Only calculate if we have a valid angle
                            // Horizontal distance calculation accounting for camera pitch angle
                            double horizontalDistance = heightDifference / Math.tan(Math.toRadians(angleToTarget));

                            // Diagonal distance from lens to center of AprilTag
                            double diagonalDistance = Math.sqrt(
                                    horizontalDistance * horizontalDistance +
                                            heightDifference * heightDifference
                            );

                            // Convert horizontal angle to inches offset (accounting for camera yaw)
                            double x_inches = horizontalDistance * Math.tan(Math.toRadians(tx_adjusted));

                            telemetryA.addData("    Angle to Target", "%.2f deg", angleToTarget);
                            telemetryA.addData("    Horizontal Dist", "%.2f inches", horizontalDistance);
                            telemetryA.addData("    Diagonal Dist", "%.2f inches", diagonalDistance);
                            telemetryA.addData("    X Offset", "%.2f inches", x_inches);
                            telemetryA.addData("    Height Diff", "%.2f inches", heightDifference);
                        } else {
                            telemetryA.addData("    Distance", "Invalid angle");
                        }

                        // Also show the 3D pose data if available
                        Pose3D targetPose = fr.getRobotPoseTargetSpace();
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

        // TODO: Drawing removed
        // Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        // TODO: Drawing removed
        // TODO: PoseUpdater removed
        // TODO: PoseUpdater removed
        // // // Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        // TODO: Drawing removed
        // Drawing.sendPacket();
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
    // Set both LED strips to the same color
    private void setLedColor(double position) {
        if (led1 != null) led1.setPosition(position);
        if (led2 != null) led2.setPosition(position);
    }
    
    /**
     * Manual relocalization from Limelight AprilTag - corrects robot pose estimate
     * Uses AprilTag detection and distance calculation
     */
    private void relocalizeFromLimelight() {
        if (limelight == null) {
            telemetryA.addData("Relocalize", "Limelight not initialized");
            return;
        }
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetryA.addData("Relocalize", "No valid data");
            return;
        }
        
        // Check latency
        double totalLatency = result.getCaptureLatency() + result.getTargetingLatency();
        if (totalLatency > MAX_LATENCY_MS) {
            telemetryA.addData("Relocalize", "Data too stale (%.1fms)", totalLatency);
            return;
        }
        
        // Get AprilTag fiducial results - only relocalize if tag is visible
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            telemetryA.addData("Relocalize", "No AprilTag visible");
            return;
        }
        
        // Use the first detected tag
        LLResultTypes.FiducialResult detectedTag = fiducialResults.get(0);
        int tagId = detectedTag.getFiducialId();
        
        // Find the tag's known field position
        double tagFieldX_Limelight = 0;
        double tagFieldY_Limelight = 0;
        boolean tagPositionKnown = false;
        
        for (double[] tagPos : APRILTAG_POSITIONS) {
            if ((int)tagPos[0] == tagId) {
                tagFieldX_Limelight = tagPos[1];
                tagFieldY_Limelight = tagPos[2];
                tagPositionKnown = true;
                break;
            }
        }
        
        if (!tagPositionKnown) {
            telemetryA.addData("Relocalize", "Tag ID %d not in known positions", tagId);
            return;
        }
        
        // Get angles from Limelight (relative to camera's view)
        double tx_raw = detectedTag.getTargetXDegrees();  // Horizontal angle from camera center
        double ty_raw = detectedTag.getTargetYDegrees();  // Vertical angle from camera center
        
        // Account for camera yaw offset (if camera is not pointing straight forward)
        // If camera is rotated right (positive yaw), tx needs to be adjusted
        double tx = tx_raw - CAMERA_YAW_DEG;
        
        // Calculate total vertical angle to target (camera pitch + ty from Limelight)
        double angleToTarget = CAMERA_PITCH_DEG + ty_raw;
        if (Math.abs(angleToTarget) < 0.5) {
            telemetryA.addData("Relocalize", "Angle too small");
            return;
        }
        
        // Calculate distance to tag (horizontal distance in the plane)
        // This accounts for the camera's pitch angle
        double horizontalDistance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
        
        // Calculate horizontal offset (left/right) from camera center
        // This accounts for the camera's yaw offset
        double xOffset = horizontalDistance * Math.tan(Math.toRadians(tx));
        
        // Robot position relative to tag (tag is at 0,0 in its own coordinate system)
        double robotX_relativeToTag = -xOffset;
        double robotY_relativeToTag = -horizontalDistance;
        
        // Convert to absolute field position
        double robotX_Limelight = tagFieldX_Limelight + robotX_relativeToTag;
        double robotY_Limelight = tagFieldY_Limelight + robotY_relativeToTag;
        
        // Convert to Pinpoint coordinates
        double pinpointX = robotX_Limelight + 72.0;
        double pinpointY = robotY_Limelight + 72.0;
        
        // Bounds check
        if (pinpointX < -10.0 || pinpointX > 154.0 || pinpointY < -10.0 || pinpointY > 154.0) {
            telemetryA.addData("Relocalize", "Pose out of bounds (%.1f, %.1f)", pinpointX, pinpointY);
            return;
        }
        
        // Keep current heading (could be improved with tag orientation)
        double pinpointHeading = follower.getPose().getHeading();
        
        // Set pose
        try {
            follower.setPose(new Pose(pinpointX, pinpointY, pinpointHeading));
            telemetryA.addData("Relocalize", "✓ Success (Tag %d, %.1fms)", tagId, totalLatency);
            telemetryA.addData("  Distance", "%.1f inches", horizontalDistance);
        } catch (Exception e) {
            telemetryA.addData("Relocalize Error", e.getMessage());
        }
    }
    
    /**
     * Attempt automatic relocalization from AprilTag detection
     * Only works when AprilTag is visible
     * Calculates distance to tag and uses known tag position to determine robot position
     * Returns true if successful
     */
    private boolean attemptRelocalizationFromAprilTag() {
        if (limelight == null) {
            return false;
        }
        
        // Get current pose to compare against
        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return false;  // No valid Limelight data
        }
        
        // Check latency
        double totalLatency = result.getCaptureLatency() + result.getTargetingLatency();
        if (totalLatency > MAX_LATENCY_MS) {
            return false;  // Data too stale
        }
        
        // Get AprilTag fiducial results - only relocalize if tag is visible
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            return false;  // No AprilTag visible
        }
        
        // Use the first detected tag (or you could prioritize specific tags)
        LLResultTypes.FiducialResult detectedTag = fiducialResults.get(0);
        int tagId = detectedTag.getFiducialId();
        
        // Find the tag's known field position (this is the tag's absolute position on field)
        double tagFieldX_Limelight = 0;
        double tagFieldY_Limelight = 0;
        boolean tagPositionKnown = false;
        
        for (double[] tagPos : APRILTAG_POSITIONS) {
            if ((int)tagPos[0] == tagId) {
                tagFieldX_Limelight = tagPos[1];  // Tag's X position in Limelight coords (center=0)
                tagFieldY_Limelight = tagPos[2];  // Tag's Y position in Limelight coords (center=0)
                tagPositionKnown = true;
                break;
            }
        }
        
        if (!tagPositionKnown) {
            return false;  // Tag ID not in our known positions
        }
        
        // Calculate robot position relative to tag (treating tag as origin 0,0)
        // Get angles from Limelight (relative to camera's view)
        double tx_raw = detectedTag.getTargetXDegrees();  // Horizontal angle (left/right, negative = tag left)
        double ty_raw = detectedTag.getTargetYDegrees();  // Vertical angle (up/down, positive = tag up)
        
        // Account for camera yaw offset (if camera is not pointing straight forward)
        double tx = tx_raw - CAMERA_YAW_DEG;
        
        // Calculate distance to tag using trigonometry
        // Total vertical angle = camera pitch + ty from Limelight
        double angleToTarget = CAMERA_PITCH_DEG + ty_raw;
        if (Math.abs(angleToTarget) < 0.5) {
            return false;  // Angle too small for accurate calculation
        }
        
        // Horizontal distance from camera to tag (in inches)
        // This accounts for the camera's pitch angle
        double horizontalDistance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
        
        // Calculate horizontal offset (left/right) from tx angle
        // This accounts for the camera's yaw offset
        // tx: negative = tag is to the left of camera center, positive = tag is to the right
        double xOffset = horizontalDistance * Math.tan(Math.toRadians(tx));
        
        // Camera position relative to tag (treating tag as 0,0)
        // This gives us where the CAMERA is, not the robot center
        // If tag is straight ahead (tx=0), camera is directly behind tag by horizontalDistance
        // If tag is to the left (tx negative), camera is to the right of tag
        // If tag is to the right (tx positive), camera is to the left of tag
        // In tag's coordinate system (tag at 0,0):
        //   - Camera is at (-xOffset, -horizontalDistance) relative to tag
        double cameraX_relativeToTag = -xOffset;  // Negative because if tag is left, camera is right
        double cameraY_relativeToTag = -horizontalDistance;  // Negative because camera is behind tag
        
        // Convert camera position to absolute field position
        double cameraX_Limelight = tagFieldX_Limelight + cameraX_relativeToTag;
        double cameraY_Limelight = tagFieldY_Limelight + cameraY_relativeToTag;
        
        // Account for camera offset from robot center
        // Camera offset is in robot-relative coordinates (forward/right)
        // Need to rotate by robot heading to convert to field coordinates
        // Get current heading to rotate the offset
        double robotHeading = currentPose.getHeading();
        
        // Rotate camera offset from robot-relative to field-relative coordinates
        // Forward offset: +X in field coords when heading = 0 (east)
        // Right offset: +Y in field coords when heading = 0 (east)
        double offsetX_field = CAMERA_FORWARD_OFFSET_INCHES * Math.cos(robotHeading) - 
                               CAMERA_RIGHT_OFFSET_INCHES * Math.sin(robotHeading);
        double offsetY_field = CAMERA_FORWARD_OFFSET_INCHES * Math.sin(robotHeading) + 
                               CAMERA_RIGHT_OFFSET_INCHES * Math.cos(robotHeading);
        
        // Robot center position = camera position - offset (camera is offset from center)
        double robotX_Limelight = cameraX_Limelight - offsetX_field;
        double robotY_Limelight = cameraY_Limelight - offsetY_field;
        
        // Convert from Limelight coordinates (center=0, meters) to Pinpoint (corner=0, inches)
        // Limelight uses center as origin, Pinpoint uses bottom-left corner
        // Field is 144" x 144", so center is at (72, 72) in Pinpoint
        double pinpointX = robotX_Limelight + 72.0;
        double pinpointY = robotY_Limelight + 72.0;
        
        // Bounds check
        if (pinpointX < -10.0 || pinpointX > 154.0 || pinpointY < -10.0 || pinpointY > 154.0) {
            return false;  // Position out of bounds
        }
        
        // For heading, we can estimate from the tag's orientation or use current heading
        // For now, keep current heading (could be improved with tag orientation)
        double pinpointHeading = currentHeading;
        
        // Check if pose change is too large (likely bad data)
        double poseChange = Math.sqrt(
            Math.pow(pinpointX - currentX, 2) + 
            Math.pow(pinpointY - currentY, 2)
        );
        
        // Reject if change is too large (prevents bad relocalization from moving turret)
        if (poseChange > MAX_POSE_CHANGE_INCHES) {
            return false;  // Pose change too large, likely bad calculation
        }
        
        // Set pose
        try {
            follower.setPose(new Pose(pinpointX, pinpointY, pinpointHeading));
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    @Override
    public void stop() {
        // Stop all motors on OpMode stop
        shootr.setPower(0);
        shootl.setPower(0);
        intakefront.setPower(0);
        intakeback.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}

