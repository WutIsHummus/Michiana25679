package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
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

@Config
@TeleOp(name = "Blueshootwhilemoving")
public class Blueshootwhilemoving extends OpMode {

    private Follower follower;
    private Telemetry telemetryA;

    // Drive
    private DcMotorEx fl, fr, bl, br;

    // Shooter + intake
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    // Servos
    private Servo turret1, turret2;
    private Servo reargate, launchgate, hood1;
    private Servo led1, led2;

    // Limelight
    private Limelight3A limelight;

    // Shooter PIDF
    private PIDFController shooterPID;

    // === Field / target constants (blue mirrored) ===
    public static double targetX = 144.0 - 128.0;
    public static double targetY = 125.0;

    public static double goalZoneX = 144.0 - 116.0;
    public static double goalZoneY = 116.0;

    public static double SNAP_X = 144.0 - 101.3293;
    public static double SNAP_Y = 123.7003;
    public static double SNAP_HEADING_DEG = (180.0 - 359.0 + 360.0) % 360.0;

    // Turret trim
    public static double turretTrimDeg = 0.0;
    public static double TRIM_STEP_DEG = 3.0;

    // Limelight geometry
    public static double aprilTagHeight = 30.0;
    public static double limelightHeight = 13.5;
    public static double heightDifference = aprilTagHeight - limelightHeight;
    public static double limelightMountAngle = 19.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition   = 0.15;
    public static double turretRightPosition  = 0.85;
    public static double turretMaxAngle       = 140.0;

    // Shooter constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;

    // Short range PIDF (< 6 ft)
    public static double p = 0.0015;
    public static double i = 0.0;
    public static double d = 0.0000;
    public static double f = 0.0009;
    public static double kV = 0.0008;
    public static double kS = 0.0;
    public static double I_ZONE = 250.0;

    // Long range PIDF (>= 6 ft)
    public static double pLong = 0.0015;
    public static double iLong = 0.0;
    public static double dLong = 0.0;
    public static double fLong = 0.00089;
    public static double kVLong = 0.0008;
    public static double kSLong = 0.0;
    public static double I_ZONE_LONG = 250.0;

    // RPM regression
    private static final double RPM_SLOPE     = 80.0;
    private static final double RPM_INTERCEPT = 1050.0;
    public static double FAR_SHOOTING_RPM_MAX = 1900.0;

    // Hood regression
    public static double HOOD_MIN_POS = 0.45;
    public static double HOOD_MAX_POS = 0.54;
    public static double HOOD_MIN_DIST_FT = 0.5;
    public static double HOOD_MAX_DIST_FT = 7.0;

    // Turret backlash
    public static double TURRET1_BACKLASH_OFFSET = 0.021;

    // Low-power mode (RT)
    public static double LOW_POWER_RPM = 800.0;
    public static double LOW_POWER_TRIGGER_THRESHOLD = 0.1;

    // Auto-shoot (A button)
    private boolean lastA = false;
    private boolean shooting = false;
    private int shootState = 0;
    private ElapsedTime shootTimer;

    // Auto-transfer (LT close zone)
    private boolean autoTransfer = false;
    private int transferState = 0;
    private boolean lastLeftTrigger = false;
    private ElapsedTime transferTimer;

    // Far LT single shot
    private boolean farSingleShot = false;
    private ElapsedTime farShotTimer;

    // Shooter on/off
    public static double RPM_TOLERANCE = 100.0;
    private boolean shootingconstant = true;
    private boolean lastY = false;

    // Voltage comp
    private static final double NOMINAL_VOLTAGE = 12.0;

    // LEDs
    public static double LED_OFF    = 0.0;
    public static double LED_RED    = 0.277;
    public static double LED_YELLOW = 0.388;
    public static double LED_GREEN  = 0.500;
    public static double LED_BLUE   = 0.611;

    // Buttons
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // Shooting-while-moving config
    public static double CLOSE_SHOT_TIME = 0.55;       // seconds
    public static double MIN_MOVING_SPEED_IPS = 5.0;   // inches / second

    // Hood default
    public static double hood1Position = 0.54;

    // Runtime + last pose for velocity computation
    private ElapsedTime runtime;
    private double lastXPos, lastYPos;
    private double lastVelUpdateTime;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        try {
            if (PoseStore.hasSaved()) {
                follower.setStartingPose(PoseStore.lastPose);
            } else {
                follower.setStartingPose(new Pose(0, 0, 0));
            }
        } catch (Exception ignored) {
            follower.setStartingPose(new Pose(0, 0, 0));
        }

        follower.startTeleopDrive();

        // Drive motors
        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // Shooter + intake
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");

        shootl.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeback.setDirection(DcMotorSimple.Direction.REVERSE);
        intakefront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        reargate = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        hood1 = hardwareMap.get(Servo.class, "hood 1");
        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        shooterPID = new PIDFController(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);

        shootTimer = new ElapsedTime();
        transferTimer = new ElapsedTime();
        farShotTimer = new ElapsedTime();
        runtime = new ElapsedTime();

        // Initial servo positions
        launchgate.setPosition(0.5);
        hood1.setPosition(hood1Position);
        setLedColor(LED_GREEN);

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.setMsTransmissionInterval(11);

        // Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
            telemetryA.addLine("Limelight initialized successfully");
        } catch (Exception e) {
            telemetryA.addLine("Warning: Limelight not found - " + e.getMessage());
            limelight = null;
        }

        // Seed last pose for velocity calc
        Pose startPose = follower.getPose();
        lastXPos = startPose.getX();
        lastYPos = startPose.getY();
        lastVelUpdateTime = runtime.seconds();

        telemetryA.addLine("Blue shoot-while-moving initialized");
        telemetryA.update();
    }

    @Override
    public void loop() {
        // Turret trim
        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean bPressed = gamepad1.b;
        if (bPressed && !lastB) {
            turretTrimDeg = 0.0;
        }
        lastB = bPressed;

        if (dpadRight && !lastDpadRight) turretTrimDeg += TRIM_STEP_DEG;
        if (dpadLeft  && !lastDpadLeft)  turretTrimDeg -= TRIM_STEP_DEG;
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;

        boolean lowPowerMode = gamepad1.right_trigger > LOW_POWER_TRIGGER_THRESHOLD;

        // Update follower & pose
        follower.update();
        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();

        // ---- Compute velocity from pose delta ----
        double now = runtime.seconds();
        double dt = now - lastVelUpdateTime;
        if (dt <= 0.0) dt = 1e-3; // avoid div/0

        double vX = (currentX - lastXPos) / dt;
        double vY = (currentY - lastYPos) / dt;
        double robotSpeed = Math.hypot(vX, vY);

        lastXPos = currentX;
        lastYPos = currentY;
        lastVelUpdateTime = now;

        // Distance to goal (for RPM + close zone)
        double deltaGoalX = goalZoneX - currentX;
        double deltaGoalY = goalZoneY - currentY;
        double distanceToGoalInches = Math.hypot(deltaGoalX, deltaGoalY);
        double distanceToGoalFeet   = distanceToGoalInches / 12.0;

        boolean isCloseZone     = distanceToGoalFeet < 7.0;
        boolean leftTriggerDown = gamepad1.left_trigger > 0.1;
        boolean isMoving        = robotSpeed > MIN_MOVING_SPEED_IPS;
        boolean movingShot      = leftTriggerDown && isCloseZone && isMoving;

        // Snap-to-pose (X)
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastX) {
            double snapHeadingRad = Math.toRadians(SNAP_HEADING_DEG);
            follower.setPose(new Pose(SNAP_X, SNAP_Y, snapHeadingRad));
        }
        lastX = xPressed;

        // Drive: slow to 50% when movingShot
        double driveScale = movingShot ? 0.5 : 1.0;
        double y = -gamepad1.right_stick_y * driveScale;
        double x =  gamepad1.right_stick_x * 1.1 * driveScale;
        double rx = gamepad1.left_stick_x * driveScale;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        fl.setPower((y + x + rx) / denom);
        bl.setPower((y - x + rx) / denom);
        fr.setPower((y - x - rx) / denom);
        br.setPower((y + x - rx) / denom);

        // ===== Velocity-compensated aim points =====
        double aimTargetX = targetX;
        double aimTargetY = targetY;
        double hoodGoalX  = goalZoneX;
        double hoodGoalY  = goalZoneY;

        if (movingShot) {
            double shotTime = CLOSE_SHOT_TIME;
            double compensatedGoalX = goalZoneX - vX * shotTime;
            double compensatedGoalY = goalZoneY - vY * shotTime;

            hoodGoalX = compensatedGoalX;
            hoodGoalY = compensatedGoalY;

            aimTargetX = targetX - vX * shotTime;
            aimTargetY = targetY - vY * shotTime;
        }

        // ===== Turret aiming (using aimTargetX/Y) =====
        double deltaX = aimTargetX - currentX;
        double deltaY = aimTargetY - currentY;
        double distance = Math.hypot(deltaX, deltaY);

        double angleToTargetField = Math.atan2(deltaY, deltaX);
        double turretAngle = angleToTargetField - currentHeading;

        while (turretAngle > Math.PI)  turretAngle -= 2 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;

        double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDegrees));
        double servoPosition;
        if (clampedAngle >= 0) {
            double range = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * range;
        } else {
            double range = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * range;
        }

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        // ===== Intake bumpers =====
        if (gamepad1.left_bumper) {
            intakeback.setPower(1.0);
        } else if (!autoTransfer && !farSingleShot) {
            intakeback.setPower(0.0);
        }

        if (gamepad1.right_bumper) {
            intakefront.setPower(1.0);
        } else if (!autoTransfer && !farSingleShot) {
            intakefront.setPower(0.0);
        }

        // ===== A button auto-shoot (3 shots, same as your original) =====
        double deltaGoalXForTiming = goalZoneX - currentX;
        double deltaGoalYForTiming = goalZoneY - currentY;
        double distanceFeetForTiming = Math.hypot(deltaGoalXForTiming, deltaGoalYForTiming) / 12.0;
        boolean isFarForShots = distanceFeetForTiming >= 6.0;
        double shotTimeScale = isFarForShots ? 16.0 : 1.0;

        boolean currentA = gamepad1.a;
        if (currentA && !lastA && !shooting) {
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }
        lastA = currentA;

        String shootStatus = "Ready";
        if (shooting) {
            switch (shootState) {
                case 0:
                    shootStatus = "Spinning up...";
                    if (shootTimer.seconds() > 1.0) {
                        shootState = 1;
                        shootTimer.reset();
                    }
                    break;
                case 1:
                    shootStatus = "Starting intakes...";
                    intakefront.setPower(-1.0);
                    intakeback.setPower(-1.0);
                    if (shootTimer.seconds() > 0.1) {
                        shootState = 2;
                        shootTimer.reset();
                    }
                    break;
                case 2:
                    shootStatus = "Firing shot 1/3";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2 * shotTimeScale) {
                        shootState = 3;
                        shootTimer.reset();
                    }
                    break;
                case 3:
                    shootStatus = "Reset 1/3";
                    launchgate.setPosition(0.5);
                    if (shootTimer.seconds() > 0.3 * shotTimeScale) {
                        shootState = 4;
                        shootTimer.reset();
                    }
                    break;
                case 4:
                    shootStatus = "Firing shot 2/3";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2 * shotTimeScale) {
                        shootState = 5;
                        shootTimer.reset();
                    }
                    break;
                case 5:
                    shootStatus = "Reset 2/3";
                    launchgate.setPosition(0.5);
                    if (shootTimer.seconds() > 0.3 * shotTimeScale) {
                        shootState = 6;
                        shootTimer.reset();
                    }
                    break;
                case 6:
                    shootStatus = "Firing shot 3/3";
                    launchgate.setPosition(0.8);
                    if (shootTimer.seconds() > 0.2 * shotTimeScale) {
                        shootState = 7;
                        shootTimer.reset();
                    }
                    break;
                case 7:
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

        // ===== Shooter ON/OFF (Y toggle) =====
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {
            shootingconstant = !shootingconstant;
        }
        lastY = yPressed;

        boolean shooterOn = shootingconstant || shooting;

        // ===== RPM from distance (no velocity compensation) =====
        boolean isLongRange = distanceToGoalFeet >= 6.0;

        double calculatedTargetRPM;
        if (lowPowerMode) {
            calculatedTargetRPM = LOW_POWER_RPM;
        } else {
            if (distanceToGoalFeet >= 9.0) {
                calculatedTargetRPM = FAR_SHOOTING_RPM_MAX;
            } else {
                calculatedTargetRPM = RPM_SLOPE * distanceToGoalFeet + RPM_INTERCEPT;
                calculatedTargetRPM = Math.max(1150.0,
                        Math.min(FAR_SHOOTING_RPM_MAX, calculatedTargetRPM));
            }
        }

        double currentP = isLongRange ? pLong : p;
        double currentI = isLongRange ? iLong : i;
        double currentD = isLongRange ? dLong : d;
        double currentF = isLongRange ? fLong : f;
        double currentKV = isLongRange ? kVLong : kV;
        double currentKS = isLongRange ? kSLong : kS;
        double currentIZone = isLongRange ? I_ZONE_LONG : I_ZONE;

        // ===== Hood regression (uses hoodGoalX/Y, which are compensated when movingShot) =====
        double hoodDeltaX = hoodGoalX - currentX;
        double hoodDeltaY = hoodGoalY - currentY;
        double hoodDistanceInches = Math.hypot(hoodDeltaX, hoodDeltaY);
        double hoodDistanceFeet   = hoodDistanceInches / 12.0;

        double hoodT = (hoodDistanceFeet - HOOD_MIN_DIST_FT) /
                (HOOD_MAX_DIST_FT - HOOD_MIN_DIST_FT);
        hoodT = Math.max(0.0, Math.min(1.0, hoodT));

        double currentHoodPos = HOOD_MAX_POS + hoodT * (HOOD_MIN_POS - HOOD_MAX_POS);
        currentHoodPos = Math.max(0.0, Math.min(1.0, currentHoodPos));
        hood1.setPosition(currentHoodPos);

        shooterPID.setPIDF(currentP, currentI, currentD, currentF);
        shooterPID.setIntegrationBounds(-currentIZone, currentIZone);

        // ===== Shooter PIDF + FF + voltage comp =====
        double targetTPS = rpmToTicksPerSec(calculatedTargetRPM);
        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double vAvg = 0.5 * (vR + vL);

        double shootrRPM = ticksPerSecToRPM(vR);
        double shootlRPM = ticksPerSecToRPM(vL);
        double avgRPM    = ticksPerSecToRPM(vAvg);

        double shooterPower = 0.0;
        double pidfOutput   = 0.0;
        double additionalFF = 0.0;

        if (shooterOn) {
            pidfOutput = shooterPID.calculate(vAvg, targetTPS);
            double sgn = Math.signum(targetTPS);
            additionalFF = (Math.abs(targetTPS) > 1e-6)
                    ? (currentKS * sgn + currentKV * targetTPS)
                    : 0.0;

            shooterPower = pidfOutput + additionalFF;

            if (avgRPM >= calculatedTargetRPM && shooterPower > 0) {
                shooterPower = Math.min(shooterPower, 0.5);
            }

            shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));

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

        // ===== LEFT TRIGGER behavior (far vs close; movingShot just affects drive/aim) =====
        boolean currentLeftTrigger = leftTriggerDown;
        boolean isFarForTransfer   = distanceToGoalFeet >= 7.0;

        if (currentLeftTrigger && !lastLeftTrigger) {
            if (isFarForTransfer) {
                // Far: single shot
                farSingleShot = true;
                farShotTimer.reset();
                autoTransfer = false;
                transferState = 0;
            } else if (!autoTransfer) {
                // Close: 4-ball auto-transfer (your existing behavior)
                autoTransfer = true;
                transferState = 0;
                transferTimer.reset();
            }
        }
        lastLeftTrigger = currentLeftTrigger;

        String transferStatus = "Ready";

        if (farSingleShot) {
            transferStatus = "Far single shot";
            double t = farShotTimer.seconds();
            if (t < 0.05) {
                launchgate.setPosition(0.8);
                intakeback.setPower(1.0);
                intakefront.setPower(1.0);
            } else if (t < 0.15) {
                launchgate.setPosition(0.5);
                intakeback.setPower(1.0);
                intakefront.setPower(1.0);
            } else {
                launchgate.setPosition(0.5);
                intakeback.setPower(0.0);
                intakefront.setPower(0.0);
                farSingleShot = false;
            }
        } else if (autoTransfer) {
            switch (transferState) {
                case 0:
                    transferStatus = "Fire 1 OPEN";
                    launchgate.setPosition(0.8);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.05) {
                        transferState = 1;
                        transferTimer.reset();
                    }
                    break;
                case 1:
                    transferStatus = "Fire 1 CLOSE";
                    launchgate.setPosition(0.5);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.1) {
                        transferState = 2;
                        transferTimer.reset();
                    }
                    break;
                case 2:
                    transferStatus = "Fire 2 OPEN";
                    launchgate.setPosition(0.8);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.05) {
                        transferState = 3;
                        transferTimer.reset();
                    }
                    break;
                case 3:
                    transferStatus = "Fire 2 CLOSE";
                    launchgate.setPosition(0.5);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.1) {
                        transferState = 4;
                        transferTimer.reset();
                    }
                    break;
                case 4:
                    transferStatus = "Fire 3 OPEN";
                    launchgate.setPosition(0.8);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.05) {
                        transferState = 5;
                        transferTimer.reset();
                    }
                    break;
                case 5:
                    transferStatus = "Fire 3 CLOSE";
                    launchgate.setPosition(0.5);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.1) {
                        transferState = 6;
                        transferTimer.reset();
                    }
                    break;
                case 6:
                    transferStatus = "Fire 4 OPEN";
                    launchgate.setPosition(0.8);
                    intakeback.setPower(1);
                    intakefront.setPower(1);
                    if (transferTimer.seconds() > 0.05) {
                        transferState = 7;
                        transferTimer.reset();
                    }
                    break;
                case 7:
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
        } else if (!currentLeftTrigger && !autoTransfer && !farSingleShot) {
            launchgate.setPosition(0.5);
        }

        // ===== LED logic =====
        boolean atTargetSpeed = Math.abs(avgRPM - calculatedTargetRPM) < RPM_TOLERANCE;
        boolean intakeActive  = gamepad1.left_bumper || gamepad1.right_bumper;

        double ledColor = LED_GREEN;
        if (shooterOn && !atTargetSpeed) {
            ledColor = LED_RED;
        } else if (shooting || autoTransfer || farSingleShot) {
            ledColor = LED_BLUE;
        } else if (intakeActive) {
            ledColor = LED_YELLOW;
        }
        setLedColor(ledColor);

        // ===== Telemetry =====
        telemetryA.addData("Pose", "(%.1f, %.1f)", currentX, currentY);
        telemetryA.addData("Heading", "%.1f°", Math.toDegrees(currentHeading));
        telemetryA.addData("Robot Speed", "%.2f in/s", robotSpeed);
        telemetryA.addData("Close Zone (<7ft)", isCloseZone ? "YES" : "NO");
        telemetryA.addData("Moving Shot", movingShot ? "YES" : "NO");
        telemetryA.addData("TurretAngle", "%.2f°", turretAngleDegrees);
        telemetryA.addData("TurretServo", "%.3f", servoPosition);
        telemetryA.addData("HoodDist(ft)", "%.2f", hoodDistanceFeet);
        telemetryA.addData("HoodPos", "%.2f", currentHoodPos);

        telemetryA.addLine("=== Shooter ===");
        telemetryA.addData("Shooter", shooterOn ? "RUNNING" : "STOPPED");
        telemetryA.addData("TargetRPM", "%.0f", calculatedTargetRPM);
        telemetryA.addData("AvgRPM", "%.0f", avgRPM);
        telemetryA.addData("ErrorRPM", "%.0f", calculatedTargetRPM - avgRPM);

        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        telemetryA.addData("Battery", "%.2f V", voltage);

        telemetryA.addLine("=== LT Modes ===");
        telemetryA.addData("FarSingleShot", farSingleShot ? "YES" : "NO");
        telemetryA.addData("AutoTransfer", autoTransfer ? "YES" : "NO");
        telemetryA.addData("TransferStatus", transferStatus);

        // Limelight summary (optional)
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetryA.addLine("=== Limelight ===");
                telemetryA.addData("tx", "%.2f", result.getTx());
                telemetryA.addData("ty", "%.2f", result.getTy());
                List<LLResultTypes.FiducialResult> frs = result.getFiducialResults();
                if (!frs.isEmpty()) {
                    telemetryA.addData("Tags", frs.size());
                    LLResultTypes.FiducialResult fr = frs.get(0);
                    Pose3D tp = fr.getRobotPoseTargetSpace();
                    if (tp != null && tp.getPosition() != null) {
                        telemetryA.addData("TagPoseX", "%.1f", tp.getPosition().x);
                        telemetryA.addData("TagPoseY", "%.1f", tp.getPosition().y);
                    }
                }
            } else {
                telemetryA.addData("Limelight", "No valid data");
            }
        }

        telemetryA.update();
    }

    private static double rpmToTicksPerSec(double rpm) {
        double motorRPM = rpm * GEAR_RATIO;
        return (motorRPM / 60.0) * TICKS_PER_REV;
    }

    private static double ticksPerSecToRPM(double tps) {
        double motorRPM = (tps / TICKS_PER_REV) * 60.0;
        return motorRPM / GEAR_RATIO;
    }

    private void setLedColor(double position) {
        if (led1 != null) led1.setPosition(position);
        if (led2 != null) led2.setPosition(position);
    }

    @Override
    public void stop() {
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
