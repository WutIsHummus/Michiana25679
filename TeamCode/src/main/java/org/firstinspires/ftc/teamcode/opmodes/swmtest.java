package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import java.util.List;

@Config
@Disabled
@TeleOp(name = "1 - BLUE(SWM)")
public class swmtest extends OpMode {

    private Follower follower;
    private Telemetry telemetryA;

    private DcMotorEx fl, fr, bl, br;

    private Servo turret1;
    private Servo turret2;

    private Limelight3A limelight;

    // Shooter hardware
    private DcMotorEx intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, hood2;
    private PIDFController shooterPID;

    // === Blue-Side Mirrored Field Constants ===
    public static double targetX = 144.0 - 128.0; // 16.0
    public static double targetY = 125.0;

    // Goal zone coordinates (mirrored X)
    public static double goalZoneX = 144.0 - 116.0; // 28.0
    public static double goalZoneY = 116.0;

    // Snap-to-pose (mirrored X and heading)
    public static double SNAP_X = 144.0 - 101.3293;
    public static double SNAP_Y = 123.7003;
    public static double SNAP_HEADING_DEG = (180.0 - 359.0 + 360.0) % 360.0;

    public static double turretTrimDeg = 0.0;
    public static double TRIM_STEP_DEG = 3.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition = 0.15;
    public static double turretRightPosition = 0.855;
    public static double turretMaxAngle = 137;

    // --- Turret backlash compensation ---
    public static double TURRET1_BACKLASH_OFFSET = 0.015;

    // Shooter PIDF Constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;

    // Short range PIDF (< 6 feet)
    public static double p = 0.0018;
    public static double i = 0.0;
    public static double d = 0.0000;
    public static double f = 0.0005;
    public static double kV = 0.0005;
    public static double kS = 0.0;
    public static double I_ZONE = 250.0;

    // Long range PIDF (>= 6 feet)
    public static double pLong = 0.0018;
    public static double iLong = 0.0;
    public static double dLong = 0.0;
    public static double fLong = 0.0008;
    public static double kVLong = 0.0008;
    public static double kSLong = 0.0;
    public static double I_ZONE_LONG = 250.0;

    // Linear regression for RPM calculation: RPM = (RPM_SLOPE) * feet + RPM_INTERCEPT
    private static final double RPM_SLOPE = 60;
    private static final double RPM_INTERCEPT = 790;

    // Max RPM caps
    public static double NORMAL_SHOOTING_RPM_MAX = 1325;
    public static double FAR_SHOOTING_RPM_MAX = 1325;

    // Far-zone micro regression window (inches)
    public static double FAR_ZONE_CENTER_IN = 117.0;
    public static double FAR_ZONE_HALF_WINDOW_IN = 15.0;
    public static double FAR_ZONE_RPM_HEADROOM = 100.0;

    // Far shooting toggle (D-pad UP) – RPM logic only
    private boolean farShootingEnabled = false;
    private boolean lastDpadUp = false;

    // === Hood regression ===
    public static double HOOD_MIN_POS = 0.45;
    public static double HOOD_MAX_POS = 0.54;

    public static double HOOD_MIN_DIST_FT = 1.0;
    public static double HOOD_MAX_DIST_FT = 7.0;

    // Voltage compensation
    private static final double NOMINAL_VOLTAGE = 12.0;

    // Launchgate positions
    public static double LAUNCHGATE_DOWN     = 0.50;
    public static double LAUNCHGATE_FULL_UP  = 0.85;
    public static double LAUNCHGATE_HALF     = (LAUNCHGATE_DOWN + LAUNCHGATE_FULL_UP) * 0.5;

    // === Active deceleration (overspeed braking) ===
    public static double BRAKE_RPM_THRESHOLD = 60.0;
    public static double BRAKE_MAX_POWER     = 0.25;
    public static double BRAKE_KP            = 0.0010;

    // === Shoot while moving (RT) ===
    public static double SWM_TRIGGER_THRESHOLD = 0.10;
    public static double SWM_PREP_TIME_SEC = 0.15;
    public static double SWM_DRIVE_POWER_CAP = 0.50;

    // === Continuous compensation blending ===
    public static double VEL_DEADBAND_IN_PER_S = 3.0;
    public static double VEL_FULL_IN_PER_S     = 18.0; // stronger sooner (was 30)

    // Shooter wheel model (for airtime estimate)
    public static double WHEEL_RATIO = 23.0 / 20.0; // motor->wheel
    public static double WHEEL_DIAMETER_MM = 96.0;

    // Airtime model tuning
    public static double BALL_EXIT_EFFICIENCY = 0.65;
    public static double BALL_EXIT_DELAY_SEC  = 0.035;
    public static double SWM_MIN_TOF_SEC      = 0.14;
    public static double SWM_MAX_TOF_SEC      = 0.60; // increased cap (was 0.45)

    // === NEW: Gains (applied ALWAYS via compAlpha) ===
    // Larger TOF => larger lead and radial distance correction
    public static double SWM_TOF_MULT = 1.50;
    // Direct scalar on turret lead strength
    public static double SWM_TURRET_LEAD_GAIN = 1.35;
    // Direct scalar on hood radial compensation strength
    public static double SWM_RADIAL_GAIN = 1.20;

    // === Auto-shoot (A) state machine ===
    private boolean lastA = false;
    private boolean shooting = false;
    private int shootState = 0;
    private ElapsedTime shootTimer;

    // === Transfer sequence (Left Trigger) ===
    private boolean lastLeftTrigger = false;
    private boolean autoTransfer = false;
    private int transferState = 0;
    private ElapsedTime transferTimer;

    // Shooter enable toggle (Y)
    private boolean shootingconstant = true;
    private boolean lastY = false;

    // Turret trim reset (B)
    private boolean lastB = false;

    // Snap pose (X)
    private boolean lastX = false;

    // Turret trim (Dpad L/R)
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // SWM internal
    private boolean swmActive = false;
    private boolean swmLast = false;
    private ElapsedTime swmPrepTimer;
    private boolean swmRequestedShot = false;
    private boolean swmStartedTransfer = false;

    // Pose velocity estimate (finite difference)
    private Pose lastPose = null;
    private double lastPoseTimeSec = 0.0;
    private ElapsedTime loopClock;

    // LED strips
    private Servo led1;
    private Servo led2;

    public static double LED_OFF    = 0.0;
    public static double LED_RED    = 0.277;
    public static double LED_YELLOW = 0.388;
    public static double LED_GREEN  = 0.500;
    public static double LED_BLUE   = 0.611;

    // === Auto-shoot inside shooting zone ===
    // Polygon from your zone-bounding paths
    private static final double[] SHOOT_ZONE_X = {
            72.106, 15.315, 25.737, 117.412, 128.473, 71.894
    };
    private static final double[] SHOOT_ZONE_Y = {
            71.681, 127.622, 142.724, 142.724, 127.835, 71.468
    };

    // One sequence per zone entry (re-arms when leaving)
    private boolean zoneArmed = true;

    // Allow/disable this behavior from dashboard
    public static boolean ZONE_AUTO_SHOOT_ENABLED = true;

    // If true, don't auto-trigger zone shooting while RT SWM is held (prevents conflict)
    public static boolean ZONE_AUTO_SHOOT_DISABLE_DURING_SWM = true;

    // RPM tolerance for "at target" for auto-zone trigger
    public static double ZONE_RPM_TOLERANCE = 100.0;

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

        shootTimer = new ElapsedTime();
        transferTimer = new ElapsedTime();
        swmPrepTimer = new ElapsedTime();
        loopClock = new ElapsedTime();

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

        telemetryA.addLine("Testing OpMode - Continuous Motion Compensation + SWM + Zone Auto-Shoot");
        telemetryA.update();
    }

    @Override
    public void loop() {

        // === Update localization ===
        follower.update();

        Pose currentPose = follower.getPose();
        double currentX  = currentPose.getX();
        double currentY  = currentPose.getY();
        double currentHeading = currentPose.getHeading();

        // === Velocity estimate (field inches/sec) ===
        double nowSec = loopClock.seconds();
        double vxField = 0.0, vyField = 0.0;
        if (lastPose != null) {
            double dt = nowSec - lastPoseTimeSec;
            if (dt > 1e-3) {
                vxField = (currentX - lastPose.getX()) / dt;
                vyField = (currentY - lastPose.getY()) / dt;
            }
        }
        lastPose = currentPose;
        lastPoseTimeSec = nowSec;

        double speed = Math.hypot(vxField, vyField);

        // === Continuous compensation alpha ===
        double compAlpha;
        if (speed <= VEL_DEADBAND_IN_PER_S) compAlpha = 0.0;
        else if (speed >= VEL_FULL_IN_PER_S) compAlpha = 1.0;
        else compAlpha = (speed - VEL_DEADBAND_IN_PER_S) / (VEL_FULL_IN_PER_S - VEL_DEADBAND_IN_PER_S);

        // === Far toggle (D-pad UP) ===
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp) farShootingEnabled = !farShootingEnabled;
        lastDpadUp = dpadUp;

        // === Turret trim (D-pad L/R), reset (B) ===
        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (dpadRight && !lastDpadRight) turretTrimDeg += TRIM_STEP_DEG;
        if (dpadLeft  && !lastDpadLeft)  turretTrimDeg -= TRIM_STEP_DEG;
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;

        boolean bPressed = gamepad1.b;
        if (bPressed && !lastB) turretTrimDeg = 0.0;
        lastB = bPressed;

        // === Snap pose (X) ===
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastX) {
            double snapHeadingRad = Math.toRadians(SNAP_HEADING_DEG);
            follower.setPose(new Pose(SNAP_X, SNAP_Y, snapHeadingRad));
        }
        lastX = xPressed;

        // === SWM activation (Right Trigger) ===
        swmActive = gamepad1.right_trigger > SWM_TRIGGER_THRESHOLD;

        if (swmActive && !swmLast) {
            swmPrepTimer.reset();
            swmRequestedShot = false;
            swmStartedTransfer = false;
        }

        if (!swmActive && swmLast) {
            swmRequestedShot = false;
            swmStartedTransfer = false;

            // If SWM had started transfer, stop it on release
            autoTransfer = false;
            transferState = 0;
            intakefront.setPower(0);
            intakeback.setPower(0);
        }
        swmLast = swmActive;

        // === Drive (RT caps immediately) ===
        double y  = -gamepad1.right_stick_y;
        double x  =  gamepad1.right_stick_x * 1.1;
        double rx =  gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double cap = swmActive ? SWM_DRIVE_POWER_CAP : 1.0;

        fl.setPower((y + x + rx) / denominator * cap);
        bl.setPower((y - x + rx) / denominator * cap);
        fr.setPower((y - x - rx) / denominator * cap);
        br.setPower((y + x - rx) / denominator * cap);

        // === Distance to goal zone ===
        double dxGoal = goalZoneX - currentX;
        double dyGoal = goalZoneY - currentY;
        double distGoalIn = Math.hypot(dxGoal, dyGoal);
        double distGoalFt = distGoalIn / 12.0;

        // Far zone detection for delay scaling
        double farMinIn = FAR_ZONE_CENTER_IN - FAR_ZONE_HALF_WINDOW_IN;
        double farMaxIn = FAR_ZONE_CENTER_IN + FAR_ZONE_HALF_WINDOW_IN;
        boolean inFarWindow = (distGoalIn >= farMinIn && distGoalIn <= farMaxIn);
        boolean farZoneForDelays = (farShootingEnabled && (inFarWindow || distGoalFt >= 9.0));
        double delayScale = farZoneForDelays ? 2.0 : 1.0;

        // === Airtime estimate ===
        double currentShooterRPM = ticksPerSecToRPM(0.5 * (shootr.getVelocity() + shootl.getVelocity()));
        double wheelRpm = currentShooterRPM * WHEEL_RATIO;

        double wheelDiamIn = (WHEEL_DIAMETER_MM / 25.4);
        double wheelCircIn = Math.PI * wheelDiamIn;
        double surfaceInPerSec = (wheelRpm * wheelCircIn) / 60.0;

        double effectiveInPerSec = Math.max(1.0, surfaceInPerSec * BALL_EXIT_EFFICIENCY);

        double tof = (distGoalIn / effectiveInPerSec) + BALL_EXIT_DELAY_SEC;

        // Increase flight time estimate (stronger lead) + clamp
        tof *= SWM_TOF_MULT;
        tof = clamp(tof, SWM_MIN_TOF_SEC, SWM_MAX_TOF_SEC);

        // === Decompose velocity into radial + lateral w.r.t. goal direction ===
        double ux = (distGoalIn > 1e-6) ? (dxGoal / distGoalIn) : 1.0;
        double uy = (distGoalIn > 1e-6) ? (dyGoal / distGoalIn) : 0.0;

        double vRad = vxField * ux + vyField * uy; // toward goal
        double px = -uy;
        double py =  ux;
        double vLat = vxField * px + vyField * py; // sideways

        // === RPM calculation (unchanged) ===
        double calculatedTargetRPM;
        double clampMax = farShootingEnabled ? FAR_SHOOTING_RPM_MAX : NORMAL_SHOOTING_RPM_MAX;

        if (farShootingEnabled && inFarWindow) {
            double baseRpmAtCenter = FAR_SHOOTING_RPM_MAX;
            double rpmPerInch = RPM_SLOPE / 12.0;
            calculatedTargetRPM = baseRpmAtCenter + rpmPerInch * (distGoalIn - FAR_ZONE_CENTER_IN);
            double farWindowMax = FAR_SHOOTING_RPM_MAX + FAR_ZONE_RPM_HEADROOM;
            calculatedTargetRPM = clamp(calculatedTargetRPM, 600, farWindowMax);
        } else if (farShootingEnabled && distGoalFt >= 9.0) {
            calculatedTargetRPM = FAR_SHOOTING_RPM_MAX;
        } else {
            calculatedTargetRPM = RPM_SLOPE * distGoalFt + RPM_INTERCEPT;
            calculatedTargetRPM = clamp(calculatedTargetRPM, 600, clampMax);
        }

        boolean isLongRange = distGoalFt >= 6.0;

        double currentP     = isLongRange ? pLong : p;
        double currentI     = isLongRange ? iLong : i;
        double currentD     = isLongRange ? dLong : d;
        double currentF     = isLongRange ? fLong : f;
        double currentKV    = isLongRange ? kVLong : kV;
        double currentKS    = isLongRange ? kSLong : kS;
        double currentIZone = isLongRange ? I_ZONE_LONG : I_ZONE;

        // === Hood (radial compensation; blended by compAlpha; gains applied ALWAYS) ===
        double hoodDistIn = distGoalIn - (compAlpha * SWM_RADIAL_GAIN * (vRad * tof));
        hoodDistIn = Math.max(0.0, hoodDistIn);
        double hoodDistFt = hoodDistIn / 12.0;

        double hoodT = (hoodDistFt - HOOD_MIN_DIST_FT) / (HOOD_MAX_DIST_FT - HOOD_MIN_DIST_FT);
        hoodT = clamp(hoodT, 0.0, 1.0);

        double currentHoodPos = HOOD_MAX_POS + hoodT * (HOOD_MIN_POS - HOOD_MAX_POS);
        currentHoodPos = clamp(currentHoodPos, 0.0, 1.0);

        hood1.setPosition(currentHoodPos);
        hood2.setPosition(currentHoodPos);

        // === Turret aiming (lateral lead; blended by compAlpha; gains applied ALWAYS) ===
        double dxAim = targetX - currentX;
        double dyAim = targetY - currentY;

        double leadAlpha = compAlpha * SWM_TURRET_LEAD_GAIN;

        double shiftX = - (leadAlpha * vLat * px * tof);
        double shiftY = - (leadAlpha * vLat * py * tof);

        dxAim += shiftX;
        dyAim += shiftY;

        double angleToTargetField = Math.atan2(dyAim, dxAim);
        double turretAngle = angleToTargetField - currentHeading;

        while (turretAngle > Math.PI)  turretAngle -= 2 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;
        double clampedAngle = clamp(turretAngleDegrees, -turretMaxAngle, turretMaxAngle);

        double servoPosition;
        if (clampedAngle >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        double turret1Pos = clamp(servoPosition + TURRET1_BACKLASH_OFFSET, 0.0, 1.0);
        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        // === Y toggles shooterconstant ===
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) shootingconstant = !shootingconstant;
        lastY = yPressed;

        // Shooter always on if constant/shooting/transfer/SWM
        boolean shooterOn = shootingconstant || shooting || autoTransfer || swmActive;

        // === Shooter closed-loop (with braking) ===
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
            if (overspeedRPM > BRAKE_RPM_THRESHOLD) {
                shooterPower += -Math.min(BRAKE_MAX_POWER, BRAKE_KP * overspeedRPM);
            }

            if (avgVelocityRPM >= calculatedTargetRPM && shooterPower > 0) {
                shooterPower = Math.min(shooterPower, 0.5);
            }

            shooterPower = clamp(shooterPower, -BRAKE_MAX_POWER, 1.0);

            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double compensatedPower = clamp(shooterPower * (NOMINAL_VOLTAGE / voltage), -1.0, 1.0);

            shootr.setPower(compensatedPower);
            shootl.setPower(compensatedPower);
        } else {
            shootr.setPower(0);
            shootl.setPower(0);
            shooterPID.reset();
        }

        boolean atTargetSpeed = Math.abs(avgVelocityRPM - calculatedTargetRPM) < ZONE_RPM_TOLERANCE;

        // === Shooting zone auto-trigger (intakes MUST spin) ===
        boolean insideShootZone = pointInPolygon(currentX, currentY, SHOOT_ZONE_X, SHOOT_ZONE_Y);

        // Re-arm when leaving the zone
        if (!insideShootZone) zoneArmed = true;

        boolean swmBlockingZone = (ZONE_AUTO_SHOOT_DISABLE_DURING_SWM && swmActive);

        if (ZONE_AUTO_SHOOT_ENABLED
                && insideShootZone
                && zoneArmed
                && atTargetSpeed
                && !shooting
                && !autoTransfer
                && !swmBlockingZone) {

            // Spin BOTH intakes immediately on zone-trigger
            intakefront.setPower(-1.0);
            intakeback.setPower(-1.0);

            // Start the shoot sequence immediately (skip waiting/spinup state)
            shooting = true;
            shootState = 1;
            shootTimer.reset();

            zoneArmed = false;
        }

        // === Auto-shoot (A) kept ===
        boolean currentA = gamepad1.a;
        if (currentA && !lastA && !shooting) {
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }
        lastA = currentA;

        // === Shoot sequence ===
        if (shooting) {
            switch (shootState) {
                case 0:
                    if (shootTimer.seconds() > 1.0 * delayScale) { shootState = 1; shootTimer.reset(); }
                    break;

                case 1:
                    // Spin BOTH intakes
                    intakefront.setPower(-1.0);
                    intakeback.setPower(-1.0);
                    if (shootTimer.seconds() > 0.1 * delayScale) { shootState = 2; shootTimer.reset(); }
                    break;

                case 2:
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (shootTimer.seconds() > 0.2 * delayScale) { shootState = 3; shootTimer.reset(); }
                    break;

                case 3:
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.3 * delayScale) { shootState = 4; shootTimer.reset(); }
                    break;

                case 4:
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (shootTimer.seconds() > 0.2 * delayScale) { shootState = 5; shootTimer.reset(); }
                    break;

                case 5:
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.3 * delayScale) { shootState = 6; shootTimer.reset(); }
                    break;

                case 6:
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (shootTimer.seconds() > 0.2 * delayScale) { shootState = 7; shootTimer.reset(); }
                    break;

                case 7:
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    intakefront.setPower(0);
                    intakeback.setPower(0);
                    if (shootTimer.seconds() > 0.2 * delayScale) {
                        shooting = false;
                        shootState = 0;
                    }
                    break;
            }
        }

        // === Manual intakes ONLY when not autoTransfer AND not shooting ===
        // (Prevents overriding intake power during zone auto-shoot / A-shoot)
        if (!autoTransfer && !shooting) {
            if (gamepad1.left_bumper) intakeback.setPower(1.0);
            else intakeback.setPower(0);

            if (gamepad1.right_bumper) intakefront.setPower(1.0);
            else intakefront.setPower(0);
        }

        // === Left Trigger starts transfer (unchanged) ===
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.1;
        if (currentLeftTrigger && !lastLeftTrigger && !autoTransfer) {
            autoTransfer = true;
            transferState = 0;
            transferTimer.reset();
        }
        lastLeftTrigger = currentLeftTrigger;

        // === Right Trigger starts SAME transfer after prep time ===
        if (swmActive && !autoTransfer) {
            if (!swmRequestedShot && swmPrepTimer.seconds() >= SWM_PREP_TIME_SEC) {
                swmRequestedShot = true;
            }
            if (swmRequestedShot && !swmStartedTransfer) {
                autoTransfer = true;
                transferState = 0;
                transferTimer.reset();
                swmStartedTransfer = true;
            }
        }

        // === Transfer state machine (your sequence) ===
        if (autoTransfer) {
            double tReq = 0.1 * delayScale;

            switch (transferState) {
                case 0:
                    intakeback.setPower(1.0);
                    intakefront.setPower(0.0);
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (transferTimer.seconds() >= tReq) { transferState = 1; transferTimer.reset(); }
                    break;

                case 1:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() >= tReq) { transferState = 2; transferTimer.reset(); }
                    break;

                case 2:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (transferTimer.seconds() >= tReq) { transferState = 3; transferTimer.reset(); }
                    break;

                case 3:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() >= tReq) { transferState = 4; transferTimer.reset(); }
                    break;

                case 4:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (transferTimer.seconds() >= tReq) { transferState = 5; transferTimer.reset(); }
                    break;

                case 5:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() >= tReq) { transferState = 6; transferTimer.reset(); }
                    break;

                case 6:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (transferTimer.seconds() >= tReq) { transferState = 7; transferTimer.reset(); }
                    break;

                case 7:
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() >= tReq) { transferState = 8; transferTimer.reset(); }
                    break;

                case 8:
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    intakeback.setPower(0.0);
                    intakefront.setPower(0.0);
                    autoTransfer = false;
                    transferState = 0;

                    // One burst per RT press
                    if (!swmActive) {
                        swmRequestedShot = false;
                        swmStartedTransfer = false;
                    }
                    break;
            }
        } else {
            if (!shooting) launchgate.setPosition(LAUNCHGATE_DOWN);
        }

        // === LEDs ===
        boolean intakeActive = (!autoTransfer && !shooting) && (gamepad1.left_bumper || gamepad1.right_bumper);

        double ledColor = LED_GREEN;
        if (shooterOn && !atTargetSpeed) ledColor = LED_RED;
        else if (shooting || autoTransfer || swmActive) ledColor = LED_BLUE;
        else if (intakeActive) ledColor = LED_YELLOW;
        setLedColor(ledColor);

        // === Telemetry ===
        telemetryA.addData("Pose", "x=%.1f y=%.1f h=%.1f", currentX, currentY, Math.toDegrees(currentHeading));
        telemetryA.addData("Vel (in/s)", "vx=%.1f vy=%.1f spd=%.1f", vxField, vyField, speed);
        telemetryA.addData("CompAlpha", "%.2f (dead=%.1f full=%.1f)", compAlpha, VEL_DEADBAND_IN_PER_S, VEL_FULL_IN_PER_S);

        telemetryA.addData("vRad/vLat", "%.1f / %.1f", vRad, vLat);
        telemetryA.addData("TOF", "%.3f (mult=%.2f clamp=%.2f..%.2f)", tof, SWM_TOF_MULT, SWM_MIN_TOF_SEC, SWM_MAX_TOF_SEC);
        telemetryA.addData("LeadGain/RadGain", "%.2f / %.2f", SWM_TURRET_LEAD_GAIN, SWM_RADIAL_GAIN);

        telemetryA.addData("InsideZone", insideShootZone);
        telemetryA.addData("ZoneArmed", zoneArmed);
        telemetryA.addData("AtTarget", atTargetSpeed);

        telemetryA.addData("RT SWM", swmActive);
        telemetryA.addData("SWM prep", "%.2f/%.2f", swmPrepTimer.seconds(), SWM_PREP_TIME_SEC);

        telemetryA.addData("TargetRPM", "%.0f", calculatedTargetRPM);
        telemetryA.addData("ActualRPM", "%.0f", avgVelocityRPM);
        telemetryA.addData("HoodPos", "%.3f", currentHoodPos);

        // Limelight telemetry unchanged
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetryA.addData("LL tx", "%.2f", result.getTx());
                telemetryA.addData("LL ty", "%.2f", result.getTy());
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                telemetryA.addData("LL tags", fiducials.size());
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    Pose3D targetPose = fr.getRobotPoseTargetSpace();
                    telemetryA.addData("Tag", fr.getFiducialId());
                    if (targetPose != null && targetPose.getPosition() != null) {
                        telemetryA.addData("TagPose", "x=%.2f y=%.2f z=%.2f",
                                targetPose.getPosition().x, targetPose.getPosition().y, targetPose.getPosition().z);
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

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Ray-casting point-in-polygon
    private static boolean pointInPolygon(double x, double y, double[] polyX, double[] polyY) {
        boolean inside = false;
        int n = polyX.length;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            boolean intersect = ((polyY[i] > y) != (polyY[j] > y)) &&
                    (x < (polyX[j] - polyX[i]) * (y - polyY[i]) / (polyY[j] - polyY[i] + 1e-9) + polyX[i]);
            if (intersect) inside = !inside;
        }
        return inside;
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
