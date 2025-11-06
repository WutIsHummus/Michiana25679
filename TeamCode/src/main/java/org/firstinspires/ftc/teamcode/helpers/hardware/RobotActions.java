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
 * REGRESSION-BASED SHOOTING EXAMPLES:
 * ====================================
 * 
 * 1. SPIN UP ONLY (No aiming, no shooting):
 *    actions.spinUpForDistance(5.5);             // From distance in feet
 *    actions.spinUpFromFollower(follower);       // From current position
 *    actions.spinUpFromPose(poseUpdater);        // From current position
 *    actions.spinUpFromPosition(x, y);           // From coordinates
 * 
 * 2. SINGLE SHOT (Auto-calculates RPM, aims turret):
 *    actions.shootFromPose(poseUpdater);
 *    actions.shootFromFollower(follower);
 *    actions.shootFromPosition(x, y, heading);
 * 
 * 3. THREE-BALL SEQUENCE (Auto-calculates RPM, aims turret):
 *    actions.threeBallFromPose(poseUpdater);
 *    actions.threeBallFromFollower(follower);
 *    actions.threeBallFromPosition(x, y, heading);
 * 
 * 4. MANUAL CONTROL:
 *    actions.aimAndShoot(1500, 0.54, 45.0);      // RPM, hood, turret angle
 *    actions.threeBallSequence(1800, 0.45, true); // RPM, hood, isLongRange
 * 
 * RPM REGRESSION FORMULA:
 * =======================
 * - Distance < 7 feet: RPM = 100 * feet + 1150 (max 1950)
 * - Distance >= 7 feet: RPM = 1950 (fixed)
 * - Hood: 0.54 (short range), 0.45 (long range)
 * - Turret: Auto-calculated from robot position
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
    
    // PID Constants - Short Range
    public static double p = 0.002;
    public static double i = 0.0;
    public static double d = 0.0001;
    public static double f = 0.00084;
    public static double kV = 0.0008;
    public static double kS = 0.01;
    
    // PID Constants - Long Range
    public static double pLong = 0.01;
    public static double iLong = 0.0;
    public static double dLong = 0.0001;
    public static double fLong = 0.00084;
    public static double kVLong = 0.0008;
    public static double kSLong = 0.01;
    
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
    
    // Hood positions
    public static double HOOD_SHORT_RANGE = 0.54;     // < 6 feet
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
                shooter.spinUp(),
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
        return new SequentialAction(
                hood.setPosition(hoodPosition),
                turret.setAngle(turretAngle),
                shooter.spinToRPM(targetRPM, true),
                shooter.waitForSpeed(targetRPM),
                launch.fire(),
                new SleepAction(0.2),
                launch.reset(),
                shooter.stop()
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
        
        // Determine hood position
        double hoodPosition = (distanceFeet >= 6.0) ? HOOD_LONG_RANGE : HOOD_SHORT_RANGE;
        
        // Execute shooting sequence
        return aimAndShoot(targetRPM, hoodPosition, turretAngleDegrees);
    }
    
    /**
     * REGRESSION-BASED THREE-BALL SEQUENCE
     * Calculates RPM, aims turret, and shoots 3 balls
     * 
     * @param robotX Current robot X position (inches)
     * @param robotY Current robot Y position (inches)
     * @param robotHeading Current robot heading (radians)
     */
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
        double hoodPosition = (distanceFeet >= 6.0) ? HOOD_LONG_RANGE : HOOD_SHORT_RANGE;
        boolean isLongRange = distanceFeet >= LONG_RANGE_THRESHOLD_FEET;
        
        // Execute sequence with turret aiming first
        return new SequentialAction(
                turret.setAngle(turretAngleDegrees),
                threeBallSequence(targetRPM, hoodPosition, isLongRange)
        );
    }
    
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
        
        // Determine hood position
        double hoodPosition = (distanceFeet >= 6.0) ? HOOD_LONG_RANGE : HOOD_SHORT_RANGE;
        
        // Set hood and spin up shooter (no turret, no shooting)
        return new SequentialAction(
                hood.setPosition(hoodPosition),
                shooter.spinToRPM(targetRPM, true)
        );
    }
    
    /**
     * SPIN UP FROM POSITION - Calculates distance automatically
     * 
     * @param robotX Current robot X position (inches)
     * @param robotY Current robot Y position (inches)
     */
    public Action spinUpFromPosition(double robotX, double robotY) {
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double distanceInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double distanceFeet = distanceInches / 12.0;
        
        return spinUpForDistance(distanceFeet);
    }
    
    /**
     * SPIN UP FROM CURRENT POSITION - Works with PoseUpdater
     */
    public Action spinUpFromPose(PoseUpdater poseUpdater) {
        Pose currentPose = poseUpdater.getPose();
        return spinUpFromPosition(currentPose.getX(), currentPose.getY());
    }
    
    /**
     * SPIN UP FROM CURRENT POSITION - Works with Follower
     */
    public Action spinUpFromFollower(Follower follower) {
        Pose currentPose = follower.getPose();
        return spinUpFromPosition(currentPose.getX(), currentPose.getY());
    }
    
    /**
     * SHOOT FROM CURRENT POSITION - Works with PoseUpdater
     * Uses regression to calculate RPM, aims turret automatically
     * 
     * @param poseUpdater Your robot's PoseUpdater
     */
    public Action shootFromPose(PoseUpdater poseUpdater) {
        Pose currentPose = poseUpdater.getPose();
        return shootFromPosition(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
    }
    
    /**
     * SHOOT FROM CURRENT POSITION - Works with Follower
     * Uses regression to calculate RPM, aims turret automatically
     * 
     * @param follower Your robot's Follower
     */
    public Action shootFromFollower(Follower follower) {
        Pose currentPose = follower.getPose();
        return shootFromPosition(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
    }
    
    /**
     * THREE-BALL SEQUENCE FROM CURRENT POSITION - Works with PoseUpdater
     * Uses regression to calculate RPM, aims turret, shoots 3 balls
     * 
     * @param poseUpdater Your robot's PoseUpdater
     */
    public Action threeBallFromPose(PoseUpdater poseUpdater) {
        Pose currentPose = poseUpdater.getPose();
        return threeBallFromPosition(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
    }
    
    /**
     * THREE-BALL SEQUENCE FROM CURRENT POSITION - Works with Follower
     * Uses regression to calculate RPM, aims turret, shoots 3 balls
     * 
     * @param follower Your robot's Follower
     */
    public Action threeBallFromFollower(Follower follower) {
        Pose currentPose = follower.getPose();
        return threeBallFromPosition(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
    }
    
    /**
     * Three-ball rapid fire with PID control
     * @param targetRPM Target shooter RPM
     * @param hoodPosition Hood servo position
     * @param isLongRange Use long range timing if true
     */
    public Action threeBallSequence(double targetRPM, double hoodPosition, boolean isLongRange) {
        if (isLongRange) {
            return new SequentialAction(
                    hood.setPosition(hoodPosition),
                    shooter.spinToRPM(targetRPM, true),
                    shooter.waitForSpeed(targetRPM),
                    // Ball 1
                    intakeFront.run(),
                    intakeBack.run(),
                    launch.fire(),
                    new SleepAction(0.2),
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
                    shooter.stop()
            );
        } else {
            return new SequentialAction(
                    hood.setPosition(hoodPosition),
                    shooter.spinToRPM(targetRPM, true),
                    shooter.waitForSpeed(targetRPM),
                    intakeFront.run(),
                    intakeBack.run(),
                    new SleepAction(0.1),
                    // Ball 1
                    launch.fire(),
                    new SleepAction(0.2),
                    launch.reset(),
                    new SleepAction(0.3),
                    // Ball 2
                    launch.fire(),
                    new SleepAction(0.2),
                    launch.reset(),
                    new SleepAction(0.3),
                    // Ball 3
                    launch.fire(),
                    new SleepAction(0.2),
                    launch.reset(),
                    intakeFront.stop(),
                    intakeBack.stop(),
                    shooter.stop()
            );
        }
    }
    
    public Action intakeAndLaunch() {
        return new SequentialAction(
                intakeBack.run(),
                intakeFront.run(),
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
                launch.reset()
        );
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
    
    public class Shooter {
        private PIDFController pidfController = null;
        private double currentTargetRPM = 0;
        private boolean pidActive = false;
        
        public Action spinUp() {
            return new InstantAction(() -> {
                shootr.setPower(1.0);
                shootl.setPower(-1.0);
                pidActive = false;
            });
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
         * Spin to target RPM using PID control - returns immediately, runs in background
         */
        public Action spinToRPM(double targetRPM, boolean useVoltageCompensation) {
            return new Action() {
                private boolean initialized = false;
                
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        // Determine if long range based on RPM
                        boolean isLongRange = targetRPM >= 1700;
                        
                        // Initialize PIDF controller with appropriate constants
                        if (isLongRange) {
                            pidfController = new PIDFController(pLong, iLong, dLong, fLong);
                        } else {
                            pidfController = new PIDFController(p, i, d, f);
                        }
                        
                        currentTargetRPM = targetRPM;
                        pidActive = true;
                        initialized = true;
                    }
                    
                    if (!pidActive) {
                        return true; // Done
                    }
                    
                    // Calculate current RPM
                    double avgVelocity = (Math.abs(shootr.getVelocity()) + Math.abs(shootl.getVelocity())) / 2.0;
                    double avgVelocityRPM = (avgVelocity / TICKS_PER_REV) * 60.0;
                    double targetTPS = (currentTargetRPM / 60.0) * TICKS_PER_REV;
                    
                    // Calculate PIDF output
                    double pidfOutput = pidfController.calculate(avgVelocity, targetTPS);
                    
                    // Add feedforward
                    double sgn = Math.signum(targetTPS);
                    double additionalFF = (Math.abs(targetTPS) > 1e-6) ? 
                        (kS * sgn + kV * targetTPS) : 0.0;
                    
                    double shooterPower = pidfOutput + additionalFF;
                    shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
                    
                    // Voltage compensation
                    if (useVoltageCompensation && voltageSensor != null) {
                        double voltage = voltageSensor.getVoltage();
                        shooterPower = shooterPower * (NOMINAL_VOLTAGE / voltage);
                        shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
                    }
                    
                    // Set motor power
                    shootr.setPower(shooterPower);
                    shootl.setPower(-shooterPower);
                    
                    // Add telemetry
                    packet.put("Target RPM", currentTargetRPM);
                    packet.put("Current RPM", avgVelocityRPM);
                    packet.put("Power", shooterPower);
                    packet.put("At Speed", Math.abs(avgVelocityRPM - currentTargetRPM) < RPM_TOLERANCE);
                    
                    return false; // Keep running (never completes on its own)
                }
            };
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

