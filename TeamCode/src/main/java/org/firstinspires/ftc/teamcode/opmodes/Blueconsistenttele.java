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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Config
@TeleOp(name = "1 - Blue New")
public class Blueconsistenttele extends OpMode {
    private Follower follower;  // Follower includes Pinpoint localization
    private MultipleTelemetry telemetryA;

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

    private PIDFController shooterPID;

    // =========================
    // FIELD MIRRORING (across X axis)
    // Assumes 144" field coordinates with Y in [0, 144].
    // Mirror: (x, y) -> (x, 144 - y), heading -> -heading (deg: 360 - deg).
    // =========================
    public static double FIELD_SIZE_Y_IN = 144.0;

    private static double mirrorY(double y) {
        return FIELD_SIZE_Y_IN - y;
    }

    private static double mirrorHeadingDeg(double deg) {
        double out = 360.0 - deg;
        // normalize to [0, 360)
        out %= 360.0;
        if (out < 0) out += 360.0;
        return out;
    }

    // =========================
    // RED SIDE FIELD CONSTANTS (MIRRORED ACROSS X AXIS)
    // =========================
    public static double targetX = 128.0;
    public static double targetY = mirrorY(125.0);

    public static double goalZoneX = 116.0;
    public static double goalZoneY = mirrorY(116.0);

    public static double SNAP_X = 101.3293;
    public static double SNAP_Y = mirrorY(123.7003);
    public static double SNAP_HEADING_DEG = mirrorHeadingDeg(359);

    // Low-power shooter mode (MOVED to gamepad2.right_trigger to avoid conflict with far toggle)
    public static double LOW_POWER_RPM = 800.0;
    public static double LOW_POWER_TRIGGER_THRESHOLD = 0.1;

    public static double turretTrimDeg = 0.0;
    public static double TRIM_STEP_DEG = 3.0;

    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition = 0.15;
    public static double turretRightPosition = 0.855;
    public static double turretMaxAngle = 137;

    // Shooter PIDF Constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;

    public static double p = 0.0012;
    public static double i = 0.0;
    public static double d = 0.0000;
    public static double f = 0.0007;
    public static double kV = 0.0006;
    public static double kS = 0.0;
    public static double I_ZONE = 250.0;

    public static double pLong = 0.0012;
    public static double iLong = 0.0;
    public static double dLong = 0;
    public static double fLong = 0.0008;
    public static double kVLong = 0.0006;
    public static double kSLong = 0.0;
    public static double I_ZONE_LONG = 250.0;

    private boolean lastB = false;

    private static final double RPM_SLOPE = 55.0;
    private static final double RPM_INTERCEPT = 790;

    // IMPORTANT: treat this as your "minimum far RPM" baseline (requested behavior)
    public static double FAR_SHOOTING_RPM_MIN = 1360;   // renamed meaning
    public static double NORMAL_SHOOTING_RPM_MAX = 1150;

    public static double FAR_ZONE_CENTER_IN = 117.0;
    public static double FAR_ZONE_HALF_WINDOW_IN = 15.0; // +/- 15 inches
    public static double FAR_ZONE_RPM_HEADROOM = 100.0;

    // In far zone, RPM increases faster only for distances BEYOND center:
    // slope 80 RPM/ft -> 80/12 RPM per inch
    public static double FAR_ZONE_SLOPE_RPM_PER_FT = 80.0;

    // Far-shooting toggle state (MOVED to right trigger - edge triggered)
    private boolean farShootingEnabled = false;
    private boolean lastFarToggleTrigger = false;

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

    public static double HOOD_MIN_POS = 0.47;
    public static double HOOD_MAX_POS_NORMAL = 0.54;
    public static double HOOD_FAR_ZONE_POS = 0.45;

    public static double HOOD_MIN_DIST_FT = 0;
    public static double HOOD_MAX_DIST_FT = 7.0;

    public static double TURRET1_BACKLASH_OFFSET = 0.015;

    private static final double NOMINAL_VOLTAGE = 12.0;

    private boolean lastX = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private boolean lastG2Left = false;
    private boolean lastG2Up = false;
    private boolean lastG2Down = false;

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
    public static double LAUNCHGATE_TWO_FIFTHS = 0.5;

    public static final double INDEX_FRONT_RETRACTED = 0.38;
    public static final double INDEX_FRONT_EXTENDED  = 0.65;
    public static final double INDEX_BACK_RETRACTED  = 0.38;
    public static final double INDEX_BACK_EXTENDED   = 0.82;

    public static double BRAKE_RPM_THRESHOLD = 40.0;
    public static double BRAKE_MAX_POWER     = 0.25;
    public static double BRAKE_KP            = 0.0010;

    // Cached voltage sensor (optimization)
    private VoltageSensor voltageSensor;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        try {
            if (PoseStore.hasSaved()) follower.setStartingPose(PoseStore.lastPose);
            else follower.setStartingPose(new Pose(0, 0, 0));
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
        telemetryA.setMsTransmissionInterval(11);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
            telemetryA.addLine("Limelight initialized successfully");
        } catch (Exception e) {
            telemetryA.addLine("Warning: Limelight not found - " + e.getMessage());
            limelight = null;
        }

        if (indexfront != null) indexfront.setPosition(INDEX_FRONT_RETRACTED);
        if (indexback  != null) indexback.setPosition(INDEX_BACK_EXTENDED);

        // Cache voltage sensor (optimization)
        try { voltageSensor = hardwareMap.voltageSensor.iterator().next(); }
        catch (Exception ignored) { voltageSensor = null; }

        telemetryA.addLine("Cosmobots Red TeleOp - Localization + Shooter");
        telemetryA.update();
    }

    @Override
    public void loop() {
        // Cache gamepad states (minor but clean)
        final boolean dpadLeft  = gamepad1.dpad_left;
        final boolean dpadRight = gamepad1.dpad_right;

        final boolean bPressed = gamepad1.b;
        if (bPressed && !lastB) turretTrimDeg = 0.0;
        lastB = bPressed;

        // Trim
        if (dpadRight && !lastDpadRight) turretTrimDeg += TRIM_STEP_DEG;
        if (dpadLeft  && !lastDpadLeft)  turretTrimDeg -= TRIM_STEP_DEG;
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;

        // =========================
        // Far toggle moved to RIGHT TRIGGER (gamepad1) edge-trigger
        // =========================
        final boolean farToggleTrigger = gamepad1.right_trigger > 0.6;
        if (farToggleTrigger && !lastFarToggleTrigger) farShootingEnabled = !farShootingEnabled;
        lastFarToggleTrigger = farToggleTrigger;

        follower.update();

        // gamepad2 relocalize (edge-triggered)
        final boolean g2Left = gamepad2.dpad_left;
        final boolean g2Up   = gamepad2.dpad_up;
        final boolean g2Down = gamepad2.dpad_down;

        // Requested: for relocalization on up/down arrows, swap them and reverse heading.
        // "Reverse heading" implemented as +PI (turn around).
        if (g2Left && !lastG2Left) follower.setPose(new Pose(104.0, mirrorY(134.0), Math.PI)); // mirrored across X + reversed
        if (g2Up   && !lastG2Up)   follower.setPose(new Pose(10.0, 10.0, Math.PI));           // swapped from DOWN + reversed
        if (g2Down && !lastG2Down) follower.setPose(new Pose(134.5, 10.0, Math.PI));          // swapped from UP + reversed

        lastG2Left = g2Left;
        lastG2Up   = g2Up;
        lastG2Down = g2Down;

        // Snap
        final boolean xPressed = gamepad1.x;
        if (xPressed && !lastX) {
            double snapHeadingRad = Math.toRadians(SNAP_HEADING_DEG);
            follower.setPose(new Pose(SNAP_X, SNAP_Y, snapHeadingRad));
        }
        lastX = xPressed;

        // Drive
        final double y  = -gamepad1.right_stick_y;
        final double x  =  gamepad1.right_stick_x * 1.1;
        final double rx =  gamepad1.left_stick_x;

        final double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        final double flPower = (y + x + rx) / denominator;
        final double blPower = (y - x + rx) / denominator;
        final double frPower = (y - x - rx) / denominator;
        final double brPower = (y + x - rx) / denominator;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        // Pose
        Pose currentPose = follower.getPose();
        final double currentX  = currentPose.getX();
        final double currentY  = currentPose.getY();
        final double currentHeading = currentPose.getHeading();

        // =========================
        // Turret aim (mirrored target/goal already applied via constants above)
        // =========================
        final double deltaX = targetX - currentX;
        final double deltaY = targetY - currentY;

        final double angleToTargetField = Math.atan2(deltaY, deltaX);
        double turretAngle = angleToTargetField - currentHeading;

        while (turretAngle > Math.PI)  turretAngle -= 2 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

        final double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;
        final double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDegrees));

        final double servoPosition;
        if (clampedAngle >= 0) {
            final double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            final double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        // Intake manual controls (bumpers) - only when not autoTransfer
        if (!autoTransfer) {
            intakeback.setPower(gamepad1.left_bumper ? 1.0 : 0.0);
            intakefront.setPower(gamepad1.right_bumper ? 1.0 : 0.0);
        }

        // dpad_down override (only when not shooting/transfer)
        final boolean g1Down = gamepad1.dpad_down;
        if (g1Down && !shooting && !autoTransfer) {
            launchgate.setPosition(LAUNCHGATE_FULL_UP);
            intakefront.setPower(-1.0);
            intakeback.setPower(-1.0);
        }

        // Launchgate idle rule
        final boolean intakeCommanded = gamepad1.left_bumper || gamepad1.right_bumper;
        if (!shooting && !autoTransfer && !g1Down) {
            launchgate.setPosition(intakeCommanded ? LAUNCHGATE_DOWN : LAUNCHGATE_TWO_FIFTHS);
        }

        // Distance to goal (mirrored goal already applied via constants above)
        final double deltaGoalX = goalZoneX - currentX;
        final double deltaGoalY = goalZoneY - currentY;
        final double distanceToGoalInches = Math.sqrt(deltaGoalX * deltaGoalX + deltaGoalY * deltaGoalY);
        final double distanceToGoalFeet = distanceToGoalInches / 12.0;

        // Far window detection
        final double farMinIn = FAR_ZONE_CENTER_IN - FAR_ZONE_HALF_WINDOW_IN;
        final double farMaxIn = FAR_ZONE_CENTER_IN + FAR_ZONE_HALF_WINDOW_IN;
        final boolean inFarWindow = (distanceToGoalInches >= farMinIn && distanceToGoalInches <= farMaxIn);

        // Far distance detection
        final boolean inFarDistance = inFarWindow || (distanceToGoalFeet >= 9.0);

        // Timing scale
        final double shootDelayScale = farShootingEnabled ? 1.0 : 1.0;

        // A button auto-shoot
        final boolean currentA = gamepad1.a;
        if (currentA && !lastA && !shooting) {
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }
        lastA = currentA;

        if (shooting) {
            switch (shootState) {
                case 0:
                    if (shootTimer.seconds() > 0 * shootDelayScale) { shootState = 1; shootTimer.reset(); }
                    break;
                case 1:
                    intakefront.setPower(-1.0);
                    intakeback.setPower(-1.0);
                    if (shootTimer.seconds() > 0.05 * shootDelayScale) { shootState = 2; shootTimer.reset(); }
                    break;
                case 2:
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) { shootState = 3; shootTimer.reset(); }
                    break;
                case 3:
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.3 * shootDelayScale) { shootState = 4; shootTimer.reset(); }
                    break;
                case 4:
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) { shootState = 5; shootTimer.reset(); }
                    break;
                case 5:
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.3 * shootDelayScale) { shootState = 6; shootTimer.reset(); }
                    break;
                case 6:
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) { shootState = 7; shootTimer.reset(); }
                    break;
                case 7:
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.3 * shootDelayScale) { shooting = false; shootState = 8; }
                    break;
                case 8:
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) { shootState = 9; shootTimer.reset(); }
                    break;
                case 9:
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) { shooting = false; shootState = 10; }
                    break;
                case 10:
                    intakefront.setPower(0);
                    intakeback.setPower(0);
                    if (shootTimer.seconds() > 0.2 * shootDelayScale) { shooting = false; shootState = 0; }
                    break;
            }
        }

        // Toggle constant shooter
        final boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) shootingconstant = !shootingconstant;
        lastY = yPressed;

        final boolean shooterOn = shootingconstant || shooting;

        // Low power moved to gamepad2.right_trigger (to avoid conflict with far toggle)
        final boolean lowPowerMode = gamepad2.right_trigger > LOW_POWER_TRIGGER_THRESHOLD;

        // RPM limit rule
        final double rpmCapAt7Ft = RPM_SLOPE * 7.0 + RPM_INTERCEPT;

        // Target RPM calculation
        double calculatedTargetRPM;
        if (lowPowerMode) {
            calculatedTargetRPM = LOW_POWER_RPM;
        } else {
            final double clampMax = farShootingEnabled
                    ? FAR_SHOOTING_RPM_MIN + FAR_ZONE_RPM_HEADROOM
                    : Math.min(NORMAL_SHOOTING_RPM_MAX, rpmCapAt7Ft);

            if (farShootingEnabled && inFarWindow) {
                // Requested: minimum far RPM at center and for distances closer than center.
                final double baseRpmAtCenter = FAR_SHOOTING_RPM_MIN;

                if (distanceToGoalInches <= FAR_ZONE_CENTER_IN) {
                    calculatedTargetRPM = baseRpmAtCenter;  // stays the same when closer than center
                } else {
                    final double rpmPerInch = (FAR_ZONE_SLOPE_RPM_PER_FT / 12.0);
                    calculatedTargetRPM = baseRpmAtCenter + rpmPerInch * (distanceToGoalInches - FAR_ZONE_CENTER_IN);
                }

                final double farWindowMax = FAR_SHOOTING_RPM_MIN + FAR_ZONE_RPM_HEADROOM;
                calculatedTargetRPM = Math.max(600, Math.min(farWindowMax, calculatedTargetRPM));
            } else if (farShootingEnabled && distanceToGoalFeet >= 9.0) {
                // Requested: far mode holds minimum far RPM
                calculatedTargetRPM = FAR_SHOOTING_RPM_MIN;
            } else {
                calculatedTargetRPM = RPM_SLOPE * distanceToGoalFeet + RPM_INTERCEPT;
                calculatedTargetRPM = Math.max(600, Math.min(clampMax, calculatedTargetRPM));
            }

            if (!farShootingEnabled) {
                calculatedTargetRPM = Math.min(calculatedTargetRPM, rpmCapAt7Ft);
            }
        }

        final boolean isLongRange = distanceToGoalFeet >= 6.0;

        final double currentP     = isLongRange ? pLong : p;
        final double currentI     = isLongRange ? iLong : i;
        final double currentD     = isLongRange ? dLong : d;
        final double currentF     = isLongRange ? fLong : f;
        final double currentKV    = isLongRange ? kVLong : kV;
        final double currentKS    = isLongRange ? kSLong : kS;
        final double currentIZone = isLongRange ? I_ZONE_LONG : I_ZONE;

        // Hood compute
        final boolean fixedFarHoodMode = (farShootingEnabled && inFarWindow);

        final double baseHoodPos;
        if (fixedFarHoodMode) {
            baseHoodPos = HOOD_FAR_ZONE_POS; // fixed in far zone only
        } else {
            double hoodT = (distanceToGoalFeet - HOOD_MIN_DIST_FT) / (HOOD_MAX_DIST_FT - HOOD_MIN_DIST_FT);
            hoodT = Math.max(0.0, Math.min(1.0, hoodT));
            final double hoodMax = HOOD_MAX_POS_NORMAL;
            double tmp = hoodMax + hoodT * (HOOD_MIN_POS - hoodMax);
            baseHoodPos = Math.max(0.0, Math.min(1.0, tmp));
        }

        // RPM-based hood adjustment (removed in far zone)
        double avgVelocityRPM = 0.0;
        double rpmError = 0.0;
        double hoodBoost = 0.0;

        final double currentHoodPos;
        if (!fixedFarHoodMode && shooterOn) {
            final double vR = shootr.getVelocity();
            final double vL = shootl.getVelocity();
            final double vAvg = 0.5 * (vR + vL);

            avgVelocityRPM = ticksPerSecToRPM(vAvg);
            rpmError = avgVelocityRPM - calculatedTargetRPM;

            if (Math.abs(rpmError) <= RPM_TOLERANCE && rpmError > 0.0) {
                hoodBoost = (rpmError / 50.0) * 0.015;
            }

            double tmp = baseHoodPos - hoodBoost;
            currentHoodPos = Math.max(0.0, Math.min(1.0, tmp));
        } else {
            currentHoodPos = Math.max(0.0, Math.min(1.0, baseHoodPos));
        }

        hood1.setPosition(currentHoodPos);
        hood2.setPosition(currentHoodPos);

        // Shooter control (only compute PID/FF when shooterOn)
        if (shooterOn) {
            shooterPID.setPIDF(currentP, currentI, currentD, currentF);
            shooterPID.setIntegrationBounds(-currentIZone, currentIZone);

            final double targetTPS = rpmToTicksPerSec(calculatedTargetRPM);

            final double vR = shootr.getVelocity();
            final double vL = shootl.getVelocity();
            final double vAvg = 0.5 * (vR + vL);

            avgVelocityRPM = ticksPerSecToRPM(vAvg);
            rpmError = avgVelocityRPM - calculatedTargetRPM;

            double pidfOutput = shooterPID.calculate(vAvg, targetTPS);

            double sgn = Math.signum(targetTPS);
            double additionalFF = (Math.abs(targetTPS) > 1e-6) ? (currentKS * sgn + currentKV * targetTPS) : 0.0;

            double shooterPower = pidfOutput + additionalFF;

            // Overspeed brake
            double brakePower = 0.0;
            if (rpmError > BRAKE_RPM_THRESHOLD) {
                brakePower = -Math.min(BRAKE_MAX_POWER, BRAKE_KP * rpmError);
            }
            shooterPower += brakePower;

            if (avgVelocityRPM >= calculatedTargetRPM && shooterPower > 0) {
                shooterPower = Math.min(shooterPower, 0.5);
            }

            shooterPower = Math.max(-BRAKE_MAX_POWER, Math.min(1.0, shooterPower));

            // Voltage compensation using cached sensor
            double voltage = (voltageSensor != null) ? voltageSensor.getVoltage() : NOMINAL_VOLTAGE;
            double compensatedPower = shooterPower * (NOMINAL_VOLTAGE / Math.max(1.0, voltage));
            compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));

            shootr.setPower(compensatedPower);
            shootl.setPower(compensatedPower);
        } else {
            shootr.setPower(0);
            shootl.setPower(0);
            shooterPID.reset();
        }

        // LEFT TRIGGER -> transfer
        final boolean currentLeftTrigger = gamepad1.left_trigger > 0.1;
        if (currentLeftTrigger && !lastLeftTrigger && !autoTransfer) {
            autoTransfer = true;
            transferState = 0;
            transferTimer.reset();
        }
        lastLeftTrigger = currentLeftTrigger;

        if (autoTransfer) {
            switch (transferState) {
                case 0:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (transferTimer.seconds() > 0.1) { transferState = 1; transferTimer.reset(); }
                    break;
                case 1:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() > 0.1) { transferState = 2; transferTimer.reset(); }
                    break;
                case 2:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (transferTimer.seconds() > 0.1) { transferState = 3; transferTimer.reset(); }
                    break;
                case 3:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() > 0.15) { transferState = 4; transferTimer.reset(); }
                    break;
                case 4:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (transferTimer.seconds() > 0.1) { transferState = 5; transferTimer.reset(); }
                    break;
                case 5:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() > 0.1) { transferState = 6; transferTimer.reset(); }
                    break;
                case 6:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (transferTimer.seconds() > 0.1) { transferState = 7; transferTimer.reset(); }
                    break;
                case 7:
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    intakeback.setPower(0.0);
                    intakefront.setPower(0.0);
                    autoTransfer = false;
                    transferState = 0;
                    break;
            }
        }

        final boolean atTargetSpeed = shooterOn && (Math.abs(avgVelocityRPM - calculatedTargetRPM) < RPM_TOLERANCE);
        final boolean intakeActive  = (!autoTransfer) && (gamepad1.left_bumper || gamepad1.right_bumper);

        // LED rules (unchanged logic)
        double ledColor = LED_GREEN;

        if (inFarDistance && !farShootingEnabled) {
            ledColor = LED_RED;
        } else if (farShootingEnabled && !inFarDistance) {
            ledColor = LED_RED;
        } else {
            if (shooterOn && !atTargetSpeed) ledColor = LED_RED;
            else if (shooting || autoTransfer) ledColor = LED_BLUE;
            else if (intakeActive) ledColor = LED_YELLOW;
        }

        setLedColor(ledColor);

        telemetryA.addData("Pose", "x=%.1f y=%.1f h=%.1f", currentX, currentY, Math.toDegrees(currentHeading));
        telemetryA.addData("FarShootingEnabled", farShootingEnabled ? "ON" : "OFF");
        telemetryA.addData("InFarWindow", inFarWindow);
        telemetryA.addData("InFarDistance", inFarDistance);
        telemetryA.addData("TargetRPM", "%.0f", calculatedTargetRPM);
        telemetryA.addData("ActualRPM", "%.0f", avgVelocityRPM);
        telemetryA.addData("RPM Error", "%.0f", rpmError);
        telemetryA.addData("HoodBase", "%.3f", baseHoodPos);
        telemetryA.addData("HoodBoost", "%.3f", hoodBoost);
        telemetryA.addData("HoodPos", "%.3f", currentHoodPos);
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
