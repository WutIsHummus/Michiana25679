package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.helpers.hardware.optimization.LoopOptimizations.BulkCacheManager;
import org.firstinspires.ftc.teamcode.helpers.hardware.optimization.LoopOptimizations.HardwareWriteCache;
import org.firstinspires.ftc.teamcode.helpers.hardware.optimization.LoopOptimizations.TelemetryThrottler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Config
@TeleOp(name = "1 - Cosmobots - Red")
public class CosmobotsRedTeleop extends OpMode {
    private Follower follower;  // Follower includes Pinpoint localization
    private MultipleTelemetry telemetryA;
    private PIDFController shooterPID;
    private VoltageSensor voltageSensor;
    private BulkCacheManager bulkCache;
    private TelemetryThrottler telemetryThrottler;

    private DcMotorEx fl, fr, bl, br;

    private Servo turret1;
    private Servo turret2;

    private Limelight3A limelight;

    // Shooter hardware
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, hood2;

    // Indexer servos (safeindexer equivalent, direct servo.setPosition)
    private Servo indexfront;
    private Servo indexback;


    // =========================
    // RED SIDE FIELD CONSTANTS
    // =========================
    // Target for turret aim (RED: original un-mirrored)
    public static double targetX = 128.0;
    public static double targetY = 125.0;

    // Goal zone coordinates (RED: original un-mirrored)
    public static double goalZoneX = 116.0;
    public static double goalZoneY = 116.0;

    // Snap-to-pose (manual relocalize)
    public static double SNAP_X = 101.3293;
    public static double SNAP_Y = 123.7003;
    public static double SNAP_HEADING_DEG = 359;

    // Low-power shooter mode (Right Trigger)
    public static double LOW_POWER_RPM = 800.0;
    public static double LOW_POWER_TRIGGER_THRESHOLD = 0.1;

    public static double turretTrimDeg = 0.0;
    public static double TRIM_STEP_DEG = 3.0;

    // Turret servo constants (hardware-specific; NOT mirrored)
    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition = 0.15;
    public static double turretRightPosition = 0.855;
    public static double turretMaxAngle = 137;

    // Shooter PIDF Constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;

    // Short range PIDF (< 6 feet)
    public static double p = 0.0015;
    public static double i = 0.0;
    public static double d = 0.0000;
    public static double f = 0.0005;
    public static double kV = 0.0005;
    public static double kS = 0.0;
    public static double I_ZONE = 250.0;

    // Long range PIDF (>= 6 feet)
    public static double pLong = 0.0015;
    public static double iLong = 0.0;
    public static double dLong = 0;
    public static double fLong = 0.0008;
    public static double kVLong = 0.0008;
    public static double kSLong = 0.0;
    public static double I_ZONE_LONG = 250.0;

    private boolean lastB = false;

    private static final double RPM_SLOPE = 62;
    private static final double RPM_INTERCEPT = 810;

    public static double FAR_SHOOTING_RPM_MAX = 1360;
    public static double NORMAL_SHOOTING_RPM_MAX = 1150;

    public static double FAR_ZONE_CENTER_IN = 117.0;
    public static double FAR_ZONE_HALF_WINDOW_IN = 15.0; // +/- 15 inches
    public static double FAR_ZONE_RPM_HEADROOM = 100.0;

    // Far-shooting toggle state (D-pad UP)
    private boolean farShootingEnabled = false;
    private boolean lastDpadUp = false;

    // Auto-shoot state machine
    private boolean lastA = false;
    private boolean shooting = false;
    private int shootState = 0;
    private ElapsedTime shootTimer;

    // Transfer state machine (Left Trigger)
    private boolean lastLeftTrigger = false;
    private boolean autoTransfer = false;
    private int transferState = 0;
    private ElapsedTime transferTimer;

    public static double RPM_TOLERANCE = 100.0;
    private boolean shootingconstant = true;
    private boolean lastY = false;

    // ===== HOOD RULES (YOUR REQUEST) =====
    // - Normal regression should be limited to 0.52 (max)
    // - When shooting from far zone ONLY, hood should be fixed at 0.54
    public static double HOOD_MIN_POS = 0.46;
    public static double HOOD_MAX_POS_NORMAL = 0.54; // cap regression to 0.52
    public static double HOOD_FAR_ZONE_POS = 0.45;    // fixed when far-zone shooting ONLY

    public static double HOOD_MIN_DIST_FT = 0;
    public static double HOOD_MAX_DIST_FT = 7.5;

    public static double TURRET1_BACKLASH_OFFSET = 0.015;

    private static final double NOMINAL_VOLTAGE = 12.0;

    private boolean lastX = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // LED strips
    private Servo led1;
    private Servo led2;

    public static double LED_OFF    = 0.0;
    public static double LED_RED    = 0.277;
    public static double LED_YELLOW = 0.388;
    public static double LED_GREEN  = 0.500;
    public static double LED_BLUE   = 0.611;

    public static double LAUNCHGATE_DOWN     = 0.50; // full down
    public static double LAUNCHGATE_FULL_UP  = 0.88; // full up
    public static double LAUNCHGATE_HALF     = (LAUNCHGATE_DOWN + LAUNCHGATE_FULL_UP) * 0.5; // halfway up

    // 2/5 of the way between DOWN and FULL UP
    public static double LAUNCHGATE_TWO_FIFTHS = 0.5;

    // Indexer positions
    public static final double INDEX_FRONT_RETRACTED = 0.38;
    public static final double INDEX_FRONT_EXTENDED  = 0.65;
    public static final double INDEX_BACK_RETRACTED  = 0.38;
    public static final double INDEX_BACK_EXTENDED   = 0.82;

    // Active deceleration (overspeed braking)
    public static double BRAKE_RPM_THRESHOLD = 40.0;
    public static double BRAKE_MAX_POWER     = 0.25;
    public static double BRAKE_KP            = 0.0010;

    @Override
    public void init() {
        HardwareWriteCache.clear(); // Reset cached writes on init
        bulkCache = new BulkCacheManager(hardwareMap);
        telemetryThrottler = new TelemetryThrottler(8.0); // ~8 Hz telemetry
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

        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");

        reargate   = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        hood1      = hardwareMap.get(Servo.class, "hood 1");
        hood2      = hardwareMap.get(Servo.class, "hood 2");

        // Indexer servos (rename if your config differs)
        try { indexfront = hardwareMap.get(Servo.class, "indexfront"); } catch (Exception ignored) { indexfront = null; }
        try { indexback  = hardwareMap.get(Servo.class, "indexback");  } catch (Exception ignored) { indexback  = null; }

        shootl.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeback.setDirection(DcMotorSimple.Direction.REVERSE);
        intakefront.setDirection(DcMotorSimple.Direction.REVERSE);

        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        shooterPID = new PIDFController(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);

        shootTimer    = new ElapsedTime();
        transferTimer = new ElapsedTime();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        
        try { voltageSensor = hardwareMap.voltageSensor.iterator().next(); } catch (Exception ignored) { voltageSensor = null; }

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
            telemetryA.addLine("Limelight initialized successfully");
        } catch (Exception e) {
            telemetryA.addLine("Warning: Limelight not found - " + e.getMessage());
            limelight = null;
        }

        // safeindexer behavior on init
        if (indexfront != null) HardwareWriteCache.setServoPosition(indexfront, INDEX_FRONT_RETRACTED);
        if (indexback  != null) HardwareWriteCache.setServoPosition(indexback, INDEX_BACK_EXTENDED);

        telemetryA.addLine("Cosmobots Red TeleOp - Localization + Shooter");
        telemetryA.update();
    }

    @Override
    public void loop() {
        if (bulkCache != null) {
            bulkCache.clear(); // Manual bulk cache clear once per loop
        }
        follower.update(); // Update Pinpoint localization each loop
        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean bPressed = gamepad1.b;
        if (bPressed && !lastB) turretTrimDeg = 0.0;
        lastB = bPressed;

        boolean lowPowerMode = gamepad1.right_trigger > LOW_POWER_TRIGGER_THRESHOLD;

        if (dpadRight && !lastDpadRight) turretTrimDeg += TRIM_STEP_DEG;
        if (dpadLeft  && !lastDpadLeft)  turretTrimDeg -= TRIM_STEP_DEG;
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;

        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp) farShootingEnabled = !farShootingEnabled;
        lastDpadUp = dpadUp;

        
        double currentVoltage = (voltageSensor != null) ? voltageSensor.getVoltage() : NOMINAL_VOLTAGE;

        boolean xPressed = gamepad1.x;
        if (xPressed && !lastX) {
            double snapHeadingRad = Math.toRadians(SNAP_HEADING_DEG);
            follower.setPose(new Pose(SNAP_X, SNAP_Y, snapHeadingRad));
        }
        lastX = xPressed;

        double y  = -gamepad1.right_stick_y;
        double x  =  gamepad1.right_stick_x * 1.1;
        double rx =  gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        Pose currentPose = follower.getPose();
        double currentX  = currentPose.getX();
        double currentY  = currentPose.getY();
        double currentHeading = currentPose.getHeading();

        /*
        // Turret aim (disabled)
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        double angleToTargetField = Math.atan2(deltaY, deltaX);
        double turretAngle = angleToTargetField - currentHeading;

        while (turretAngle > Math.PI)  turretAngle -= 2 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;
        double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDegrees));

        double servoPosition;
        if (clampedAngle >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        HardwareWriteCache.setServoPosition(turret1, turret1Pos - 0.01);
        HardwareWriteCache.setServoPosition(turret2, servoPosition - 0.01);
        */

        // Intake manual controls (bumpers)
        if (!autoTransfer) {
            if (gamepad1.left_bumper) HardwareWriteCache.setMotorPower(intakeback, 1.0);
            else HardwareWriteCache.setMotorPower(intakeback, 0);

            if (gamepad1.right_bumper) HardwareWriteCache.setMotorPower(intakefront, 1.0);
            else HardwareWriteCache.setMotorPower(intakefront, 0);
        }

        // Launchgate idle rule
        boolean intakeCommanded = gamepad1.left_bumper || gamepad1.right_bumper;
        if (!shooting && !autoTransfer) {
            HardwareWriteCache.setServoPosition(launchgate, intakeCommanded ? LAUNCHGATE_DOWN : LAUNCHGATE_TWO_FIFTHS);
        }

        // Distance to goal for RPM / hood
        double deltaGoalX = goalZoneX - currentX;
        double deltaGoalY = goalZoneY - currentY;
        double distanceToGoalInches = Math.sqrt(deltaGoalX * deltaGoalX + deltaGoalY * deltaGoalY);
        double distanceToGoalFeet = distanceToGoalInches / 12.0;

        // Far window detection (same window you already use)
        double farMinIn = FAR_ZONE_CENTER_IN - FAR_ZONE_HALF_WINDOW_IN;
        double farMaxIn = FAR_ZONE_CENTER_IN + FAR_ZONE_HALF_WINDOW_IN;
        boolean inFarWindow = (distanceToGoalInches >= farMinIn && distanceToGoalInches <= farMaxIn);

        // Far-zone timing scale (DOUBLED)
        double shootDelayScale = farShootingEnabled ? 1.01 : 1.0;

        // A button auto-shoot (timing scaled when far)
        boolean currentA = gamepad1.a;
        if (currentA && !lastA && !shooting) {
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }
        lastA = currentA;

        if (shooting) {
            switch (shootState) {
                case 0:
                    if (shootTimer.seconds() > 0.005 * shootDelayScale) {
                        shootState = 1;
                        shootTimer.reset();
                    }
                    break;

                case 1:
                    HardwareWriteCache.setMotorPower(intakefront, -1.0);
                    HardwareWriteCache.setMotorPower(intakeback, -1.0);
                    if (shootTimer.seconds() > 0.1 * shootDelayScale) {
                        shootState = 2;
                        shootTimer.reset();
                    }
                    break;

                case 2:
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_HALF);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) {
                        shootState = 3;
                        shootTimer.reset();
                    }
                    break;

                case 3:
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.3 * shootDelayScale) {
                        shootState = 4;
                        shootTimer.reset();
                    }
                    break;

                case 4:
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_HALF);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) {
                        shootState = 5;
                        shootTimer.reset();
                    }
                    break;

                case 5:
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.3 * shootDelayScale) {
                        shootState = 6;
                        shootTimer.reset();
                    }
                    break;

                case 6:
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_HALF);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) {
                        shootState = 7;
                        shootTimer.reset();
                    }
                    break;

                case 7:
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_DOWN);

                    if (shootTimer.seconds() > 0.3 * shootDelayScale) {
                        shooting = false;
                        shootState = 8;
                    }
                    break;
                case 8:
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_FULL_UP);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) {
                        shootState = 9;
                        shootTimer.reset();
                    }
                    break;

                case 9:
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) {
                        shooting = false;
                        shootState = 10;
                    }
                    break;
                case 10:
                    HardwareWriteCache.setMotorPower(intakefront, 0);
                    HardwareWriteCache.setMotorPower(intakeback, 0);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) {
                        shooting = false;
                        shootState = 0;
                    }
                    break;
            }
        }

        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) shootingconstant = !shootingconstant;
        lastY = yPressed;

        boolean shooterOn = shootingconstant || shooting;

        // =========================
        // RPM LIMIT RULE (YOUR REQUEST)
        // =========================
        // Until far shooting is enabled (Dpad Up), cap target RPM to the RPM at 7ft.
        // (Low-power mode still overrides everything.)
        double rpmCapAt7Ft = RPM_SLOPE * 7.0 + RPM_INTERCEPT;

        // Target RPM calculation
        double calculatedTargetRPM;
        if (lowPowerMode) {
            calculatedTargetRPM = LOW_POWER_RPM;
        } else {
            double clampMax;
            if (!farShootingEnabled) {
                // Hard cap to the RPM at 7ft until far shooting is enabled
                clampMax = Math.min(NORMAL_SHOOTING_RPM_MAX, rpmCapAt7Ft);
            } else {
                clampMax = FAR_SHOOTING_RPM_MAX;
            }

            if (farShootingEnabled && inFarWindow) {
                double baseRpmAtCenter = FAR_SHOOTING_RPM_MAX;
                double rpmPerInch = RPM_SLOPE / 12.0;
                calculatedTargetRPM = baseRpmAtCenter + rpmPerInch * (distanceToGoalInches - FAR_ZONE_CENTER_IN);

                double farWindowMax = FAR_SHOOTING_RPM_MAX + FAR_ZONE_RPM_HEADROOM;
                calculatedTargetRPM = Math.max(600, Math.min(farWindowMax, calculatedTargetRPM));

            } else if (farShootingEnabled && distanceToGoalFeet >= 9.0) {
                calculatedTargetRPM = FAR_SHOOTING_RPM_MAX;

            } else {
                calculatedTargetRPM = RPM_SLOPE * distanceToGoalFeet + RPM_INTERCEPT;
                calculatedTargetRPM = Math.max(600, Math.min(clampMax, calculatedTargetRPM));
            }

            // Extra safety: if far not enabled, never exceed rpmCapAt7Ft
            if (!farShootingEnabled) {
                calculatedTargetRPM = Math.min(calculatedTargetRPM, rpmCapAt7Ft);
            }
        }

        boolean isLongRange = distanceToGoalFeet >= 6.0;

        double currentP     = isLongRange ? pLong : p;
        double currentI     = isLongRange ? iLong : i;
        double currentD     = isLongRange ? dLong : d;
        double currentF     = isLongRange ? fLong : f;
        double currentKV    = isLongRange ? kVLong : kV;
        double currentKS    = isLongRange ? kSLong : kS;
        double currentIZone = isLongRange ? I_ZONE_LONG : I_ZONE;

        // =========================
        // HOOD RULES (YOUR REQUEST)
        // =========================
        // - Normal regression should be limited to 0.52 (max)
        // - When shooting from far zone ONLY (far enabled AND in far window), set hood=0.54 fixed
        double currentHoodPos;
        if (farShootingEnabled && inFarWindow) {
            currentHoodPos = HOOD_FAR_ZONE_POS; // fixed in far zone only
        } else {
            double hoodT = (distanceToGoalFeet - HOOD_MIN_DIST_FT) / (HOOD_MAX_DIST_FT - HOOD_MIN_DIST_FT);
            hoodT = Math.max(0.0, Math.min(1.0, hoodT));

            // regression with capped max of 0.52
            double hoodMax = HOOD_MAX_POS_NORMAL;
            currentHoodPos = hoodMax + hoodT * (HOOD_MIN_POS - hoodMax);

            // clamp
            currentHoodPos = Math.max(0.0, Math.min(1.0, currentHoodPos));
        }
        HardwareWriteCache.setServoPosition(hood1, currentHoodPos);
        HardwareWriteCache.setServoPosition(hood2, currentHoodPos);

        // Shooter PID setup
        shooterPID.setPIDF(currentP, currentI, currentD, currentF);
        shooterPID.setIntegrationBounds(-currentIZone, currentIZone);

        double targetTPS = rpmToTicksPerSec(calculatedTargetRPM);

        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double vAvg = 0.5 * (vR + vL);

        double avgVelocityRPM = ticksPerSecToRPM(vAvg);

        if (shooterOn) {
            double pidfOutput = shooterPID.calculate(vAvg, targetTPS);

            double sgn = Math.signum(targetTPS);
            double additionalFF = (Math.abs(targetTPS) > 1e-6) ? (currentKS * sgn + currentKV * targetTPS) : 0.0;

            double shooterPower = pidfOutput + additionalFF;

            double overspeedRPM = avgVelocityRPM - calculatedTargetRPM;
            double brakePower = 0.0;
            if (overspeedRPM > BRAKE_RPM_THRESHOLD) {
                brakePower = -Math.min(BRAKE_MAX_POWER, BRAKE_KP * overspeedRPM);
            }
            shooterPower += brakePower;

            if (avgVelocityRPM >= calculatedTargetRPM && shooterPower > 0) {
                shooterPower = Math.min(shooterPower, 0.5);
            }

            shooterPower = Math.max(-BRAKE_MAX_POWER, Math.min(1.0, shooterPower));

            // Voltage compensation (as in your code)
            double voltage = currentVoltage;
            double compensatedPower = shooterPower * (NOMINAL_VOLTAGE / voltage);
            compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));

            HardwareWriteCache.setMotorPower(shootr, compensatedPower);
            HardwareWriteCache.setMotorPower(shootl, compensatedPower);
        } else {
            HardwareWriteCache.setMotorPower(shootr, 0);
            HardwareWriteCache.setMotorPower(shootl, 0);
            shooterPID.reset();
        }

        // LEFT TRIGGER -> transfer (unchanged)
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.1;
        if (currentLeftTrigger && !lastLeftTrigger && !autoTransfer) {
            autoTransfer = true;
            transferState = 0;
            transferTimer.reset();
        }
        lastLeftTrigger = currentLeftTrigger;

        if (autoTransfer) {
            switch (transferState) {
                case 0:
                    HardwareWriteCache.setMotorPower(intakeback, 1.0);
                    HardwareWriteCache.setMotorPower(intakefront, 1.0);
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_HALF);
                    if (transferTimer.seconds() > 0.1) { transferState = 1; transferTimer.reset(); }
                    break;

                case 1:
                    HardwareWriteCache.setMotorPower(intakeback, 1.0);
                    HardwareWriteCache.setMotorPower(intakefront, 1.0);
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() > 0.1) { transferState = 2; transferTimer.reset(); }
                    break;

                case 2:
                    HardwareWriteCache.setMotorPower(intakeback, 1.0);
                    HardwareWriteCache.setMotorPower(intakefront, 1.0);
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_HALF);
                    if (transferTimer.seconds() > 0.1) { transferState = 3; transferTimer.reset(); }
                    break;

                case 3:
                    HardwareWriteCache.setMotorPower(intakeback, 1.0);
                    HardwareWriteCache.setMotorPower(intakefront, 1.0);
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() > 0.15) { transferState = 4; transferTimer.reset(); }
                    break;

                case 4:
                    HardwareWriteCache.setMotorPower(intakeback, 1.0);
                    HardwareWriteCache.setMotorPower(intakefront, 1.0);
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_FULL_UP);
                    if (transferTimer.seconds() > 0.1) { transferState = 5; transferTimer.reset(); }
                    break;

                case 5:
                    HardwareWriteCache.setMotorPower(intakeback, 1.0);
                    HardwareWriteCache.setMotorPower(intakefront, 1.0);
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() > 0.1) { transferState = 6; transferTimer.reset(); }
                    break;

                case 6:
                    HardwareWriteCache.setMotorPower(intakeback, 1.0);
                    HardwareWriteCache.setMotorPower(intakefront, 1.0);
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_FULL_UP);
                    if (transferTimer.seconds() > 0.1) { transferState = 7; transferTimer.reset(); }
                    break;

                case 7:
                    HardwareWriteCache.setServoPosition(launchgate, LAUNCHGATE_DOWN);
                    HardwareWriteCache.setMotorPower(intakeback, 0.0);
                    HardwareWriteCache.setMotorPower(intakefront, 0.0);
                    autoTransfer = false;
                    transferState = 0;
                    break;
            }
        }

        boolean atTargetSpeed = Math.abs(avgVelocityRPM - calculatedTargetRPM) < RPM_TOLERANCE;
        boolean intakeActive  = (!autoTransfer) && (gamepad1.left_bumper || gamepad1.right_bumper);

        double ledColor = LED_GREEN;
        if (shooterOn && !atTargetSpeed) ledColor = LED_RED;
        else if (shooting || autoTransfer) ledColor = LED_BLUE;
        else if (intakeActive) ledColor = LED_YELLOW;
        setLedColor(ledColor);

        // =========================
        // CURRENT DRAW TELEMETRY (YOUR REQUEST)
        // =========================

        if (telemetryThrottler == null || telemetryThrottler.shouldUpdate()) {
            telemetryA.addData("Pose", "x=%.1f y=%.1f h=%.1f", currentX, currentY, Math.toDegrees(currentHeading));
            telemetryA.addData("FarShootingEnabled", farShootingEnabled ? "ON" : "OFF");
            telemetryA.addData("InFarWindow", inFarWindow);
            telemetryA.addData("RPM cap @7ft", "%.0f", rpmCapAt7Ft);
            telemetryA.addData("TargetRPM", "%.0f", calculatedTargetRPM);
            telemetryA.addData("ActualRPM", "%.0f", avgVelocityRPM);
            telemetryA.addData("HoodPos", "%.3f", currentHoodPos);
            telemetryA.update();
        }
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
        if (led1 != null) HardwareWriteCache.setServoPosition(led1, position);
        if (led2 != null) HardwareWriteCache.setServoPosition(led2, position);
    }

    @Override
    public void stop() {
        HardwareWriteCache.setMotorPower(shootr, 0);
        HardwareWriteCache.setMotorPower(shootl, 0);
        HardwareWriteCache.setMotorPower(intakefront, 0);
        HardwareWriteCache.setMotorPower(intakeback, 0);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}






