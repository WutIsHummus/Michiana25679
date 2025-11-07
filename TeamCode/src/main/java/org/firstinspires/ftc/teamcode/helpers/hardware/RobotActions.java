package org.firstinspires.ftc.teamcode.helpers.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * RobotActions - Reusable shooting and intake actions for FTC
 * 
 * EASIEST WAY - DISTANCE-BASED (No coordinates needed!):
 * ======================================================
 * 
 * actions.shootAtDistance(48, 0, follower);        // Shoot 48" straight ahead
 * actions.shootAtDistance(60, 45, follower);       // Shoot 60" at 45° left
 * actions.shootAtDistance(36, -90, follower);      // Shoot 36" at 90° right
 * actions.threeBallAtDistance(48, 0, follower);    // 3 balls, 48" straight ahead
 * 
 * ALSO WORKS WITH:
 * - Follower: actions.shootAtDistance(48, 0, follower)
 * - PoseUpdater: actions.shootAtDistance(48, 0, poseUpdater)
 * 
 * OTHER SHOOTING METHODS:
 * =======================
 * 
 * 1. SPIN UP ONLY:
 *    actions.spinUpForDistance(5.5);             // From distance in feet
 *    actions.spinUpFromFollower(follower);       // From current position
 * 
 * 2. FIELD COORDINATE SHOOTING:
 *    actions.shootFromPose(poseUpdater);         // To goal zone
 *    actions.shootFromFollower(follower);        // To goal zone
 *    actions.shootFromPosition(x, y, heading);   // Manual coordinates
 * 
 * 3. MANUAL CONTROL:
 *    actions.aimAndShoot(1500, 0.54, 45.0);      // RPM, hood, turret angle
 * 
 * RPM REGRESSION FORMULA:
 * =======================
 * - Distance < 7 feet: RPM = 100 * feet + 1150 (max 1950)
 * - Distance >= 7 feet: RPM = 1950 (fixed)
 * - Hood: 0.54 (short range), 0.45 (long range)
 * - Turret: Auto-calculated
 * 
 * All constants are @Config tunable on FTC Dashboard!
 */
@Config
public class RobotActions {
    
    private final DcMotor intakefront;
    private final DcMotor intakeback;
    private final DcMotorEx shootr;
    private final DcMotorEx shootl;
    private final Servo launchgate;
    private final Servo reargate;
    private final Servo hood1;
    private final Servo turret1;
    private final Servo turret2;
    private final VoltageSensor voltageSensor;
    
    public final IntakeFront intakeFront;
    public final IntakeBack intakeBack;
    public final Shooter shooter;
    public final LaunchGate launch;
    public final RearGate rear;
    public final Hood hood;
    public final Turret turret;
    
    // PID Constants - Short Range (< 6 feet)
    public static double p = 0.002;
    public static double i = 0.0;
    public static double d = 0.0001;
    public static double f = 0.0;          // NOT USED - using kV instead
    public static double kV = 0.00084;     // Velocity feedforward (replaces F term)
    public static double kS = 0.01;        // Static friction compensation
    public static double I_ZONE = 250.0;
    
    // PID Constants - Long Range (>= 6 feet)
    public static double pLong = 0.01;
    public static double iLong = 0.0;
    public static double dLong = 0.0001;
    public static double fLong = 0.0;      // NOT USED - using kV instead
    public static double kVLong = 0.00084; // Velocity feedforward (replaces F term)
    public static double kSLong = 0.01;    // Static friction compensation
    public static double I_ZONE_LONG = 250.0;
    
    // Motor constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;
    public static double RPM_TOLERANCE = 50.0;
    
    // Voltage compensation
    private static final double NOMINAL_VOLTAGE = 12.0;
    
    // RPM Regression constants
    public static double RPM_SLOPE = 100.0;           // RPM per foot
    public static double RPM_INTERCEPT = 1150.0;      // Base RPM
    public static double FAR_SHOOTING_RPM_MAX = 1950.0;  // Max RPM cap
    public static double LONG_RANGE_THRESHOLD_FEET = 7.0;  // Fixed RPM beyond this distance
    public static double PID_THRESHOLD_FEET = 6.0;    // Switch PID/hood at 6 feet (matches other files)
    
    // Hood positions
    public static double HOOD_SHORT_RANGE = 0.54;
    public static double HOOODAUTO = 0.49;// < 6 feet
    public static double HOOD_LONG_RANGE = 0.45;      // >= 6 feet
    
    // Goal coordinates
    public static double GOAL_X = 115.0;
    public static double GOAL_Y = 121.0;
    
    // Full constructor with all hardware
    public RobotActions(DcMotor intakefront, DcMotor intakeback, DcMotorEx shootr, DcMotorEx shootl, 
                        Servo launchgate, Servo reargate, Servo hood1, Servo turret1, Servo turret2,
                        VoltageSensor voltageSensor) {
        this.intakefront = intakefront;
        this.intakeback = intakeback;
        this.shootr = shootr;
        this.shootl = shootl;
        this.launchgate = launchgate;
        this.reargate = reargate;
        this.hood1 = hood1;
        this.turret1 = turret1;
        this.turret2 = turret2;
        this.voltageSensor = voltageSensor;
        
        intakeFront = new IntakeFront();
        intakeBack = new IntakeBack();
        shooter = new Shooter();
        launch = new LaunchGate();
        rear = new RearGate();
        hood = new Hood();
        turret = new Turret();
    }
    
    // Backward compatible constructor (no PID features)
    public RobotActions(DcMotor intakefront, DcMotor intakeback, DcMotor shootr, DcMotor shootl, 
                        Servo launchgate, Servo reargate) {
        this.intakefront = intakefront;
        this.intakeback = intakeback;
        this.shootr = (DcMotorEx) shootr;  // Cast to DcMotorEx
        this.shootl = (DcMotorEx) shootl;
        this.launchgate = launchgate;
        this.reargate = reargate;
        this.hood1 = null;
        this.turret1 = null;
        this.turret2 = null;
        this.voltageSensor = null;
        
        intakeFront = new IntakeFront();
        intakeBack = new IntakeBack();
        shooter = new Shooter();
        launch = new LaunchGate();
        rear = new RearGate();
        hood = new Hood();
        turret = new Turret();
    }
    
    public Action startIntake() {
        return new ParallelAction(
                intakeFront.run(),
                intakeBack.runSlow()
        );
    }
    
    public Action stopIntake() {
        return new ParallelAction(
                intakeFront.stop(),
                intakeBack.stop()
        );
    }
    
    public Action shootSequence() {
        return new SequentialAction(
                shooter.spinUp(),
                new SleepAction(0.5),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset()
        );
    }


    public Action rapidFire() {
        return new SequentialAction(
                intakeFront.run(),
                intakeBack.run(),
                new SleepAction(0.5),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                new SleepAction(0.3),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                shooter.stop()
        );
    }
    public Action launch3() {
        return new SequentialAction(
                intakeBack.run(),
                new SleepAction(0.1),
                launch.fire(),
                intakeFront.run(),
                new SleepAction(0.1),
                launch.reset(),
                new SleepAction(0.2),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                new SleepAction(0.2),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                new SleepAction(0.2),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                shooter.stop()
        );
    }









    /**
     * PID-based shooting action - spins to target RPM using PID control
     * @param targetRPM Target RPM for shooter
     * @param useVoltageCompensation Enable/disable voltage compensation
     */
    public Action shootAtRPM(double targetRPM, boolean useVoltageCompensation) {
        return shooter.spinToRPM(targetRPM, useVoltageCompensation);
    }
    
    /**
     * Complete shooting sequence with PID control
     * @param targetRPM Target shooter RPM
     * @param hoodPosition Hood servo position
     * @param turretAngle Turret angle in degrees
     */
    public Action aimAndShoot(double targetRPM, double hoodPosition, double turretAngle) {
        return new ParallelAction(
                // PID runs continuously in parallel
                shooter.spinToRPM(targetRPM, true),
                // Shooting sequence runs alongside PID
                new SequentialAction(
                        hood.setPosition(hoodPosition),
                        turret.setAngle(turretAngle),
                        shooter.waitForSpeed(targetRPM),
                        launch.fire(),
                        new SleepAction(0.2),
                        launch.reset(),
                        shooter.stop()  // This stops the PID too
                )
        );
    }
    
    /**
     * Complete shooting sequence with PID control and range info
     * @param targetRPM Target shooter RPM
     * @param hoodPosition Hood servo position
     * @param turretAngle Turret angle in degrees
     * @param isLongRange Whether this is long range (affects PID selection)
     */
    public Action aimAndShootWithRange(double targetRPM, double hoodPosition, double turretAngle, boolean isLongRange) {
        return new ParallelAction(
                // PID runs continuously in parallel with range info
                shooter.spinToRPMWithRange(targetRPM, true, isLongRange),
                // Shooting sequence runs alongside PID
                new SequentialAction(
                        hood.setPosition(hoodPosition),
                        turret.setAngle(turretAngle),
                        shooter.waitForSpeed(targetRPM),
                        launch.fire(),
                        new SleepAction(0.2),
                        launch.reset(),
                        shooter.stop()  // This stops the PID too
                )
        );
    }
    
    /**
     * REGRESSION-BASED SHOOTING - Calculate RPM from distance
     * Automatically aims turret, sets hood, spins up shooter, and fires
     * 
     * @param robotX Current robot X position (inches)
     * @param robotY Current robot Y position (inches)
     * @param robotHeading Current robot heading (radians)
     */
    public Action shootFromPosition(double robotX, double robotY, double robotHeading) {
        // Calculate distance to goal
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double distanceInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double distanceFeet = distanceInches / 12.0;
        
        // Calculate RPM using regression
        double targetRPM;
        if (distanceFeet >= LONG_RANGE_THRESHOLD_FEET) {
            targetRPM = FAR_SHOOTING_RPM_MAX;  // Fixed RPM for long range
        } else {
            targetRPM = RPM_SLOPE * distanceFeet + RPM_INTERCEPT;
            targetRPM = Math.max(1250.0, Math.min(FAR_SHOOTING_RPM_MAX, targetRPM));
        }
        
        // Calculate turret angle
        double angleToGoal = Math.atan2(deltaY, deltaX);  // Field angle to goal
        double turretAngleDegrees = Math.toDegrees(angleToGoal - robotHeading);
        
        // Normalize to -180 to 180
        while (turretAngleDegrees > 180) turretAngleDegrees -= 360;
        while (turretAngleDegrees < -180) turretAngleDegrees += 360;
        
        // Determine hood position and PID range
        boolean isLongRange = distanceFeet >= PID_THRESHOLD_FEET;
        double hoodPosition = isLongRange ? HOOD_LONG_RANGE : HOOD_SHORT_RANGE;
        
        // Execute shooting sequence with range info
        return aimAndShootWithRange(targetRPM, hoodPosition, turretAngleDegrees, isLongRange);
    }


    /**
     * INSTANT AUTO SHOOT - Shoots immediately at specified distance
     * Perfect for autonomous - just call this and it shoots automatically!
     * NO turret movement - just spins up, waits for speed, and shoots
     * 
     * @param distanceInches Distance to target in inches (e.g., 39 for 3.25 feet)
     */
    public Action autoShootOneBall(double distanceInches) {
        // Calculate distance to goal
        double distanceFeet = distanceInches / 12.0;

        // Calculate RPM using regression
        double targetRPM;
        if (distanceFeet >= LONG_RANGE_THRESHOLD_FEET) {
            targetRPM = FAR_SHOOTING_RPM_MAX;
        } else {
            targetRPM = RPM_SLOPE * distanceFeet + RPM_INTERCEPT;
            targetRPM = Math.max(1250.0, Math.min(FAR_SHOOTING_RPM_MAX, targetRPM));
        }

        // Determine hood position and range type
        boolean isLongRange = distanceFeet >= PID_THRESHOLD_FEET;
        double hoodPosition = (distanceFeet >= PID_THRESHOLD_FEET) ? HOOD_LONG_RANGE : HOOODAUTO;

        // Execute automatic shooting sequence (NO turret movement)
        return new SequentialAction(
                hood.setPosition(hoodPosition),
                new ParallelAction(
                        shooter.spinToRPMWithRange(targetRPM, true, isLongRange),
                        new SequentialAction(
                                shooter.waitForSpeed(targetRPM),
                                intakeFront.run(),
                                intakeBack.run(),
                                new SleepAction(0.1),
                                launch.fire(),
                                new SleepAction(0.3),
                                launch.reset(),
                                intakeFront.stop(),
                                intakeBack.stop(),
                                shooter.stop()
                        )
                )
        );
    }

    public Action threeBallabsoluteclose(double distance, double turretangle) {
        // Calculate distance to goal
        double distanceInches = distance;
        double distanceFeet = distanceInches / 12.0;

        // Calculate RPM using regression
        double targetRPM;
        if (distanceFeet >= LONG_RANGE_THRESHOLD_FEET) {
            targetRPM = FAR_SHOOTING_RPM_MAX;  // Fixed RPM for long range
        } else {
            targetRPM = RPM_SLOPE * distanceFeet + RPM_INTERCEPT;
            targetRPM = Math.max(1250.0, Math.min(FAR_SHOOTING_RPM_MAX, targetRPM));
        }
        // Field angle to goal
        double turretAngleDegrees = turretangle;

        // Normalize to -180 to 180
        while (turretAngleDegrees > 180) turretAngleDegrees -= 360;
        while (turretAngleDegrees < -180) turretAngleDegrees += 360;

        // Determine hood position and range type
        double hoodPosition = HOOODAUTO;
        boolean isLongRange = distanceFeet >= PID_THRESHOLD_FEET;

        // Execute sequence with turret aiming first
        return new SequentialAction(
                turret.setAngle(turretAngleDegrees),
                threeBallSequence(targetRPM, hoodPosition, isLongRange)
        );
    }
    public Action threeBallFromPosition(double robotX, double robotY, double robotHeading) {
        // Calculate distance to goal
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double distanceInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double distanceFeet = distanceInches / 12.0;
        
        // Calculate RPM using regression
        double targetRPM;
        if (distanceFeet >= LONG_RANGE_THRESHOLD_FEET) {
            targetRPM = FAR_SHOOTING_RPM_MAX;  // Fixed RPM for long range
        } else {
            targetRPM = RPM_SLOPE * distanceFeet + RPM_INTERCEPT;
            targetRPM = Math.max(1250.0, Math.min(FAR_SHOOTING_RPM_MAX, targetRPM));
        }

        // Calculate turret angle
        double angleToGoal = Math.atan2(deltaY, deltaX);  // Field angle to goal
        double turretAngleDegrees = Math.toDegrees(angleToGoal - robotHeading);
        
        // Normalize to -180 to 180
        while (turretAngleDegrees > 180) turretAngleDegrees -= 360;
        while (turretAngleDegrees < -180) turretAngleDegrees += 360;
        
        // Determine hood position and range type
        boolean isLongRange = distanceFeet >= PID_THRESHOLD_FEET;
        double hoodPosition = isLongRange ? HOOD_LONG_RANGE : HOOD_SHORT_RANGE;
        
        // Execute sequence with turret aiming first
        return new SequentialAction(
                turret.setAngle(turretAngleDegrees),
                threeBallSequence(targetRPM, hoodPosition, isLongRange)
        );
    }

    /**
     * DISTANCE-BASED SHOOTING - Robot-relative coordinates
     * Much easier than field coordinates!
     * 
     * @param distanceInches Distance to target (e.g., 48 for 48 inches away)
     * @param angleDegrees Angle relative to robot (0 = straight ahead, 90 = left, -90 = right)
     * @param robotX Current robot X (inches)
     * @param robotY Current robot Y (inches)
     * @param robotHeading Current robot heading (radians)
     */
    public Action shootAtDistance(double distanceInches, double angleDegrees, 
                                   double robotX, double robotY, double robotHeading) {
        // Convert robot-relative polar coordinates to field coordinates
        double angleRadians = Math.toRadians(angleDegrees);
        double fieldAngle = robotHeading + angleRadians;
        
        double targetX = robotX + distanceInches * Math.cos(fieldAngle);
        double targetY = robotY + distanceInches * Math.sin(fieldAngle);
        
        // Use existing field-coordinate shooting
        return shootFromPosition(targetX, targetY, robotHeading);
    }
    
    /**
     * DISTANCE-BASED THREE-BALL - Robot-relative
     * 
     * @param distanceInches Distance to target
     * @param angleDegrees Angle relative to robot (0 = straight ahead)
     * @param robotX Current robot X (inches)
     * @param robotY Current robot Y (inches)  
     * @param robotHeading Current robot heading (radians)
     */
    public Action threeBallAtDistance(double distanceInches, double angleDegrees,
                                       double robotX, double robotY, double robotHeading) {
        // Convert robot-relative polar coordinates to field coordinates
        double angleRadians = Math.toRadians(angleDegrees);
        double fieldAngle = robotHeading + angleRadians;
        
        double targetX = robotX + distanceInches * Math.cos(fieldAngle);
        double targetY = robotY + distanceInches * Math.sin(fieldAngle);
        
        // Use existing field-coordinate shooting
        return threeBallFromPosition(targetX, targetY, robotHeading);
    }
    
    /**
     * DISTANCE-BASED - Works with Follower
     * Example: actions.shootAtDistance(48, 0, follower) = Shoot 48" straight ahead
     */
    /**
     * DISTANCE-BASED THREE-BALL - Works with Follower  
     * Example: actions.threeBallAtDistance(48, 0, follower) = Shoot 3 balls 48" straight ahead
     */
    public Action threeBallAtDistance(double distanceInches, double angleDegrees, Follower follower) {
        Pose pose = follower.getPose();
        return threeBallAtDistance(distanceInches, angleDegrees, pose.getX(), pose.getY(), pose.getHeading());
    }
    
    /**
     * DISTANCE-BASED THREE-BALL - Works with PoseUpdater
     * Example: actions.threeBallAtDistance(48, 0, poseUpdater) = Shoot 3 balls 48" straight ahead
     */
    /**
     * SPIN UP SHOOTER FOR DISTANCE - Just spins up, doesn't shoot
     * Calculates RPM from distance, sets hood, NO turret aiming or firing
     *
     * @param distanceFeet Distance to target in feet
     */
    public Action spinUpForDistance(double distanceFeet) {
        // Calculate RPM using regression
        double targetRPM;
        if (distanceFeet >= LONG_RANGE_THRESHOLD_FEET) {
            targetRPM = FAR_SHOOTING_RPM_MAX;
        } else {
            targetRPM = RPM_SLOPE * distanceFeet + RPM_INTERCEPT;
            targetRPM = Math.max(1250.0, Math.min(FAR_SHOOTING_RPM_MAX, targetRPM));
        }
        
        // Determine hood position and range type
        boolean isLongRange = distanceFeet >= PID_THRESHOLD_FEET;
        double hoodPosition = isLongRange ? HOOD_LONG_RANGE : HOOD_SHORT_RANGE;
        
        // Set hood and spin up shooter (no turret, no shooting)
        return new SequentialAction(
                hood.setPosition(hoodPosition),
                shooter.spinToRPMWithRange(targetRPM, true, isLongRange)
        );
    }

    public Action threeBallSequence(double targetRPM, double hoodPosition, boolean isLongRange) {
        if (isLongRange) {
            return new ParallelAction(
                    // PID runs continuously in parallel
                    shooter.spinToRPM(targetRPM, true),
                    // Shooting sequence runs alongside PID
                    new SequentialAction(
                            hood.setPosition(hoodPosition),
                            shooter.waitForSpeed(targetRPM),
                            intakeBack.run(),
                            new SleepAction(0.2),
                            launch.fire(),
                            new SleepAction(0.1),
                            intakeFront.run(),
                            new SleepAction(0.1),
                            launch.reset(),
                            intakeFront.stop(),
                            intakeBack.stop(),
                            new SleepAction(0.3),
                            // Ball 2
                            intakeFront.run(),
                            intakeBack.run(),
                            launch.fire(),
                            new SleepAction(0.2),
                            launch.reset(),
                            intakeFront.stop(),
                            intakeBack.stop(),
                            new SleepAction(0.3),
                            // Ball 3
                            intakeFront.run(),
                            intakeBack.run(),
                            launch.fire(),
                            new SleepAction(0.2),
                            launch.reset(),
                            intakeFront.stop(),
                            intakeBack.stop(),
                            shooter.stop()  // This stops the PID too
                    )
            );
        } else {
            return new ParallelAction(
                    // PID runs continuously in parallel
                    shooter.spinToRPM(targetRPM, true),
                    // Shooting sequence runs alongside PID
                    new SequentialAction(
                            hood.setPosition(hoodPosition),
                            shooter.waitForSpeed(targetRPM),
                            intakeFront.run(),
                            intakeBack.run(),
                            new SleepAction(0.1),
                            // Ball 1
                            launch.fire(),
                            new SleepAction(0.2),
                            launch.reset(),
                            new SleepAction(0.2),
                            // Ball 2
                            launch.fire(),
                            new SleepAction(0.2),
                            launch.reset(),
                            new SleepAction(0.2),
                            // Ball 3
                            launch.fire(),
                            new SleepAction(0.2),
                            launch.reset(),
                            launch.fire(),
                            new SleepAction(0.2),
                            launch.reset(),
                            intakeFront.stop(),
                            intakeBack.stop()// This stops the PID too
                    )
            );
        }
    }

    
    public Action safePositions() {
        return new ParallelAction(
                intakeFront.stop(),
                intakeBack.stop(),
                shooter.stop(),
                launch.reset(),
                rear.close()
        );
    }
    
    public class IntakeFront {
        public Action run() {
            return new InstantAction(() -> intakefront.setPower(-1.0));
        }
        
        public Action runSlow() {
            return new InstantAction(() -> intakefront.setPower(-0.5));
        }
        
        public Action reverse() {
            return new InstantAction(() -> intakefront.setPower(1.0));
        }
        
        public Action stop() {
            return new InstantAction(() -> intakefront.setPower(0.0));
        }
    }
    
    public class IntakeBack {
        public Action run() {
            return new InstantAction(() -> intakeback.setPower(-1.0));
        }
        
        public Action runSlow() {
            return new InstantAction(() -> intakeback.setPower(-0.5));
        }
        
        public Action reverse() {
            return new InstantAction(() -> intakeback.setPower(1.0));
        }
        
        public Action stop() {
            return new InstantAction(() -> intakeback.setPower(0.0));
        }
    }
    public Action holdShooterAtRPMclose(double targetRPM, double holdSeconds) {
        return new SequentialAction(
                // Set hood down
                hood.setPosition(HOOD_LONG_RANGE),

                // Run PIDF control while holding for specified time
                new ParallelAction(
                        shooter.spinToRPMWithRange(targetRPM, true, false),
                        new SequentialAction(
                                shooter.waitForSpeed(targetRPM),
                                new SleepAction(holdSeconds)
                        )
                ),

                // Stop shooter after hold
                shooter.stop()
        );
    }
    public class Shooter {
        private PIDFController pidfController = null;
        private double currentTargetRPM = 0;
        private volatile boolean pidActive = false;  // Use volatile for thread-safety
        private long lastPIDCallTime = 0;
        private Action currentPIDAction = null;  // Track the current PID action
        
        public Action spinUp() {
            return new InstantAction(() -> {
                shootr.setPower(1.0);
                shootl.setPower(-1.0);
                pidActive = false;
            });
        }

        // --- Add inside Shooter class ---
        public double getCurrentRPM() {
            // Only shootr has the encoder; convert ticks/sec -> RPM
            double ticksPerSec = Math.abs(shootr.getVelocity());
            return (ticksPerSec / TICKS_PER_REV) * 60.0;
        }

        public Action spinUpSlow() {
            return new InstantAction(() -> {
                shootr.setPower(0.7);
                shootl.setPower(-0.7);
                pidActive = false;
            });
        }
        
        public Action stop() {
            return new InstantAction(() -> {
                shootr.setPower(0.0);
                shootl.setPower(0.0);
                pidActive = false;
                if (pidfController != null) {
                    pidfController.reset();
                }
            });
        }

        /**
         * Hold shooter at a target RPM for a specified time with PIDF control.
         * Hood stays down (long-range position). Voltage compensation is always on.
         *
         * @param targetRPM Shooter speed in RPM
         * @param holdSeconds Duration to hold that speed (seconds)
         */



        /**
         * Spin to target RPM using PID control - returns immediately, runs in background
         * @deprecated Use spinToRPMWithRange instead for proper PID selection
         */
        public Action spinToRPM(double targetRPM, boolean useVoltageCompensation) {
            // Fallback: use RPM threshold to guess range (not ideal but maintains compatibility)
            boolean isLongRange = targetRPM >= 1700;
            return spinToRPMWithRange(targetRPM, useVoltageCompensation, isLongRange);
        }
        
        /**
         * Spin to target RPM using PID control with explicit range selection
         * @param targetRPM Target RPM for shooter
         * @param useVoltageCompensation Enable voltage compensation
         * @param isLongRange True for long range (>=6ft), false for short range (<6ft)
         */
        public Action spinToRPMWithRange(double targetRPM, boolean useVoltageCompensation, boolean isLongRange) {
            // If there's already a PID action running, stop it first
            if (pidActive && currentPIDAction != null) {
                pidActive = false;
                if (pidfController != null) {
                    pidfController.reset();
                }
            }
            
            Action newAction = new Action() {
                private boolean initialized = false;
                private double currentP, currentI, currentD, currentF, currentKV, currentKS, currentIZone;
                
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    // Prevent multiple simultaneous PID calls
                    long currentTime = System.nanoTime();
                    if (initialized && (currentTime - lastPIDCallTime) < 5_000_000) {  // 5ms minimum interval
                        return false;  // Skip this call, too soon
                    }
                    lastPIDCallTime = currentTime;
                    
                    if (!initialized) {
                        // Use appropriate PID values based on distance
                        if (isLongRange) {
                            currentP = pLong;
                            currentI = iLong;
                            currentD = dLong;
                            currentF = fLong;
                            currentKV = kVLong;
                            currentKS = kSLong;
                            currentIZone = I_ZONE_LONG;
                        } else {
                            currentP = p;
                            currentI = i;
                            currentD = d;
                            currentF = f;
                            currentKV = kV;
                            currentKS = kS;
                            currentIZone = I_ZONE;
                        }
                        
                        // Initialize official FTCLib PIDF controller (F=0 since we use kV manually)
                        pidfController = new PIDFController(currentP, currentI, currentD, currentF);
                        pidfController.setIntegrationBounds(-currentIZone, currentIZone);
                        
                        currentTargetRPM = targetRPM;
                        pidActive = true;
                        initialized = true;
                    }
                    
                    if (!pidActive) {
                        return true; // Done
                    }
                    
                    // Convert target RPM to ticks per second
                    double targetTPS = (currentTargetRPM / 60.0) * TICKS_PER_REV;
                    
                    // Read current velocities
                    double vR = Math.abs(shootr.getVelocity());
                    double vL = Math.abs(shootl.getVelocity());
                    double vAvg = 0.5 * (vR + vL);
                    
                    // Convert to RPM for display
                    double avgVelocityRPM = (vAvg / TICKS_PER_REV) * 60.0;
                    
                    double shooterPower = 0;
                    double pidfOutput = 0;
                    double additionalFF = 0;
                    
                    // PIDF control (F term is built-in and multiplied by setpoint)
                    pidfOutput = pidfController.calculate(vAvg, targetTPS);
                    
                    // Additional feedforward using kV and kS (F=0, so we use kV manually)
                    double sgn = Math.signum(targetTPS);
                    additionalFF = (Math.abs(targetTPS) > 1e-6) ? (currentKS * sgn + currentKV * targetTPS) : 0.0;
                    
                    // Total power (PIDF output already includes F*setpoint)
                    shooterPower = pidfOutput + additionalFF;
                    
                    // Safety: prevent overshoot (EXACTLY like FullTesting line 464)
                    if (avgVelocityRPM >= currentTargetRPM && shooterPower > 0) {
                        shooterPower = Math.min(shooterPower, 0.5);
                    }
                    
                    // Clamp
                    shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
                    
                    // Voltage compensation (EXACTLY like FullTesting line 472)
                    if (useVoltageCompensation && voltageSensor != null) {
                        double voltage = voltageSensor.getVoltage();
                        double compensatedPower = shooterPower * (NOMINAL_VOLTAGE / voltage);
                        compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));
                        
                        shootr.setPower(compensatedPower);
                        shootl.setPower(compensatedPower);
                    } else {
                        shootr.setPower(shooterPower);
                        shootl.setPower(shooterPower);
                    }
                    
                    // Add telemetry
                    packet.put("Target RPM", currentTargetRPM);
                    packet.put("Current RPM", avgVelocityRPM);
                    packet.put("PIDF Output", pidfOutput);
                    packet.put("Additional FF", additionalFF);
                    packet.put("Total Power", shooterPower);
                    packet.put("PID Range", isLongRange ? "LONG" : "SHORT");
                    packet.put("At Speed", Math.abs(avgVelocityRPM - currentTargetRPM) < RPM_TOLERANCE);
                    
                    return false; // Keep running (never completes on its own)
                }
            };
            
            currentPIDAction = newAction;
            return newAction;
        }
        
        /**
         * Wait for shooter to reach target speed
         */
        public Action waitForSpeed(double targetRPM) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    double avgVelocity = (Math.abs(shootr.getVelocity()) + Math.abs(shootl.getVelocity())) / 2.0;
                    double avgVelocityRPM = (avgVelocity / TICKS_PER_REV) * 60.0;
                    
                    boolean atSpeed = Math.abs(avgVelocityRPM - targetRPM) < RPM_TOLERANCE;
                    
                    packet.put("Waiting for Speed", !atSpeed);
                    packet.put("Current RPM", avgVelocityRPM);
                    packet.put("Target RPM", targetRPM);
                    
                    return atSpeed; // Done when at speed
                }
            };
        }
    }
    
    public class Hood {
        public Action setPosition(double position) {
            return new InstantAction(() -> {
                if (hood1 != null) hood1.setPosition(position);
            });
        }
        
        public Action shortRange() {
            return new InstantAction(() -> {
                if (hood1 != null) hood1.setPosition(0.54);
            });
        }
        
        public Action longRange() {
            return new InstantAction(() -> {
                if (hood1 != null) hood1.setPosition(0.45);
            });
        }
    }
    
    public class Turret {
        public double turretCenterPosition = 0.51;
        public double turretLeftPosition = 0.275;
        public double turretRightPosition = 0.745;
        public double turretMaxAngle = 90.0;
        
        public Action setAngle(double angleDegrees) {
            return new InstantAction(() -> {
                if (turret1 == null || turret2 == null) return;
                
                double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, angleDegrees));
                double servoPosition;
                
                if (clampedAngle >= 0) {
                    servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * 
                        (turretRightPosition - turretCenterPosition);
                } else {
                    servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * 
                        (turretCenterPosition - turretLeftPosition);
                }
                
                turret1.setPosition(servoPosition);
                turret2.setPosition(servoPosition);
            });
        }
        
        public Action center() {
            return new InstantAction(() -> {
                if (turret1 != null && turret2 != null) {
                    turret1.setPosition(turretCenterPosition);
                    turret2.setPosition(turretCenterPosition);
                }
            });
        }
    }
    
    public class LaunchGate {
        public Action fire() {
            return new InstantAction(() -> launchgate.setPosition(0.8));
        }
        
        public Action reset() {
            return new InstantAction(() -> launchgate.setPosition(0.5));
        }
        
        public Action open() {
            return new InstantAction(() -> launchgate.setPosition(1.0));
        }
        
        public Action close() {
            return new InstantAction(() -> launchgate.setPosition(0.0));
        }
    }
    
    public class RearGate {
        public Action open() {
            return new InstantAction(() -> reargate.setPosition(1.0));
        }
        
        public Action close() {
            return new InstantAction(() -> reargate.setPosition(0.0));
        }
        
        public Action middle() {
            return new InstantAction(() -> reargate.setPosition(0.5));
        }
    }
}

