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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Config
@TeleOp(name = "1 - Testingblue")
public class Testingblue extends OpMode {
    private Follower follower;  // Follower includes Pinpoint localization
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
    // NOTE: We'll aim turret using SWM-compensated goal derived from goalZoneX/Y.
    public static double targetX = 144.0 - 128.0; // kept (not used for turret after SWM change)
    public static double targetY = 125.0;         // kept (not used for turret after SWM change)

    // Goal zone coordinates (mirrored X)
    public static double goalZoneX = 144.0 - 116.0; // mirror of Red-side 116.0 → 28.0
    public static double goalZoneY = 116.0;

    // Snap-to-pose (mirrored X and heading)
    public static double SNAP_X = 144.0 - 101.3293; // mirror of Red-side snap point → 42.6707
    public static double SNAP_Y = 123.7003;
    public static double SNAP_HEADING_DEG = (180.0 - 359.0 + 360.0) % 360.0; // mirrors 359° → 181°

    public static double turretTrimDeg = 0.0;
    public static double TRIM_STEP_DEG = 3.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition = 0.15;
    public static double turretRightPosition = 0.855;
    public static double turretMaxAngle = 134;

    // Shooter PIDF Constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0; // motor-to-encoder ratio (leave as-is; your RPM math is motor-RPM based)

    // === Motor -> Flywheel ratio (you said 23:20 motor -> flywheel) ===
    // FlywheelRPM = MotorRPM * (23/20)
    public static double MOTOR_TO_FLYWHEEL_RATIO = 23.0 / 20.0;

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
    public static double dLong = 0;
    public static double fLong = 0.0008;
    public static double kVLong = 0.0008;
    public static double kSLong = 0.0;
    public static double I_ZONE_LONG = 250.0;

    private boolean lastB = false;

    private static final double RPM_SLOPE = 60;
    private static final double RPM_INTERCEPT = 790;

    public static double FAR_SHOOTING_RPM_MAX = 1325;
    public static double NORMAL_SHOOTING_RPM_MAX = 1325;

    public static double FAR_ZONE_CENTER_IN = 117.0;
    public static double FAR_ZONE_HALF_WINDOW_IN = 15.0; // +/- 15 inches
    public static double FAR_ZONE_RPM_HEADROOM = 100.0;

    // Far-shooting toggle state (D-pad UP)
    private boolean farShootingEnabled = false;
    private boolean lastDpadUp = false;

    // === RIGHT TRIGGER = Shoot While Moving (hold) ===
    public static double SHOOT_TRIGGER_THRESHOLD = 0.10;
    public static double DRIVE_SHOOT_SCALE = 0.70; // limit drivetrain speed while shooting

    // Auto-shoot state machine (now driven by right trigger hold)
    private boolean shooting = false;
    private int shootState = 0;
    private ElapsedTime shootTimer;

    // Transfer state machine (Left Trigger)
    private boolean lastLeftTrigger = false;
    private boolean autoTransfer = false;
    private int transferState = 0;
    private ElapsedTime transferTimer;

    public static double RPM_TOLERANCE = 100.0;
    private boolean shootingconstant = true; // unchanged (Y toggles)
    private boolean lastY = false;

    public static double HOOD_MIN_POS = 0.45;
    public static double HOOD_MAX_POS = 0.54;

    public static double HOOD_MIN_DIST_FT = 1.0;
    public static double HOOD_MAX_DIST_FT = 7.0;

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
    public static double LAUNCHGATE_FULL_UP  = 0.85; // full up
    public static double LAUNCHGATE_HALF     = (LAUNCHGATE_DOWN + LAUNCHGATE_FULL_UP) * 0.5; // halfway up

    // Active deceleration (overspeed braking)
    public static double BRAKE_RPM_THRESHOLD = 60.0;
    public static double BRAKE_MAX_POWER     = 0.25;
    public static double BRAKE_KP            = 0.0010;

    // === SWM Airtime model (tunable) ===
    // airtimeSeconds ≈ distanceInches / (AIRTIME_SPEED_PER_FLYWHEEL_RPM * flywheelRPM)
    // You MUST tune AIRTIME_SPEED_PER_FLYWHEEL_RPM so airtime feels right.
    public static double AIRTIME_SPEED_PER_FLYWHEEL_RPM = 0.080; // inches/sec per flywheel RPM (tune)
    public static double AIRTIME_MIN_S = 0.15;
    public static double AIRTIME_MAX_S = 1.20;
    public static int    AIRTIME_ITERATIONS = 3;

    // Pose delta velocity estimation
    private Pose lastPoseForVel = null;
    private ElapsedTime velTimer = new ElapsedTime();

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

        lastPoseForVel = null;
        velTimer.reset();

        telemetryA.addLine("Full Testing OpMode - Localization + Shooter (SWM enabled)");
        telemetryA.update();
    }

    @Override
    public void loop() {
        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean bPressed = gamepad1.b;
        if (bPressed && !lastB) turretTrimDeg = 0.0;
        lastB = bPressed;

        if (dpadRight && !lastDpadRight) turretTrimDeg += TRIM_STEP_DEG;
        if (dpadLeft  && !lastDpadLeft)  turretTrimDeg -= TRIM_STEP_DEG;
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;

        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp) farShootingEnabled = !farShootingEnabled;
        lastDpadUp = dpadUp;

        follower.update();

        boolean xPressed = gamepad1.x;
        if (xPressed && !lastX) {
            double snapHeadingRad = Math.toRadians(SNAP_HEADING_DEG);
            follower.setPose(new Pose(SNAP_X, SNAP_Y, snapHeadingRad));
        }
        lastX = xPressed;

        // =========================================================
        // Drivetrain (ONLY change: right trigger limits speed to 0.7)
        // =========================================================
        boolean shootTriggerHeld = gamepad1.right_trigger > SHOOT_TRIGGER_THRESHOLD;
        double driveScale = shootTriggerHeld ? DRIVE_SHOOT_SCALE : 1.0;

        double y  = -gamepad1.right_stick_y * driveScale;
        double x  =  gamepad1.right_stick_x * 1.1 * driveScale;
        double rx =  gamepad1.left_stick_x * driveScale;

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

        // =========================================================
        // Estimate robot field velocity (inches/sec) from pose delta
        // =========================================================
        double dt = velTimer.seconds();
        velTimer.reset();

        double vx = 0.0;
        double vy = 0.0;

        if (lastPoseForVel != null && dt > 1e-3) {
            vx = (currentX - lastPoseForVel.getX()) / dt;
            vy = (currentY - lastPoseForVel.getY()) / dt;
        }
        lastPoseForVel = currentPose;

        // =========================================================
        // SWM: velocity-compensated goal position via 3 iterations
        // =========================================================
        SwmSolution swm = computeSwmCompensatedGoal(
                currentX, currentY,
                goalZoneX, goalZoneY,
                vx, vy,
                farShootingEnabled
        );

        double goalXComp = swm.goalXComp;
        double goalYComp = swm.goalYComp;
        double distanceToGoalInches = swm.distanceInchesComp;
        double distanceToGoalFeet = distanceToGoalInches / 12.0;

        // =========================================================
        // Turret aiming now uses SWM-compensated goal (always active)
        // =========================================================
        double deltaX = goalXComp - currentX;
        double deltaY = goalYComp - currentY;

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

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        // Manual intake only when not transferring (unchanged)
        if (!autoTransfer) {
            if (gamepad1.left_bumper) intakeback.setPower(1.0);
            else intakeback.setPower(0);

            if (gamepad1.right_bumper) intakefront.setPower(1.0);
            else intakefront.setPower(0);
        }

        // =========================================================
        // Right trigger hold-to-shoot looping (replaces low power mode)
        // =========================================================
        if (shootTriggerHeld && !shooting) {
            // Start immediately on press: intakes on + begin firing sequence right away.
            shooting = true;
            shootState = 2;           // jump straight to "Firing shot 1/3"
            shootTimer.reset();
            intakefront.setPower(-1.0);
            intakeback.setPower(-1.0);
            launchgate.setPosition(LAUNCHGATE_HALF);
        } else if (!shootTriggerHeld && shooting) {
            // Stop immediately on release
            shooting = false;
            shootState = 0;
            launchgate.setPosition(LAUNCHGATE_DOWN);
            intakefront.setPower(0);
            intakeback.setPower(0);
        }

        String shootStatus = "Ready";
        if (shooting) {
            // Keep intakes running while shooting (per your prior behavior)
            intakefront.setPower(-1.0);
            intakeback.setPower(-1.0);

            switch (shootState) {
                case 2:
                    shootStatus = "Firing shot 1/3";
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (shootTimer.seconds() > 0.20) { shootState = 3; shootTimer.reset(); }
                    break;

                case 3:
                    shootStatus = "Reset 1/3";
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.30) { shootState = 4; shootTimer.reset(); }
                    break;

                case 4:
                    shootStatus = "Firing shot 2/3";
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (shootTimer.seconds() > 0.20) { shootState = 5; shootTimer.reset(); }
                    break;

                case 5:
                    shootStatus = "Reset 2/3";
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (shootTimer.seconds() > 0.30) { shootState = 6; shootTimer.reset(); }
                    break;

                case 6:
                    shootStatus = "Firing shot 3/3";
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (shootTimer.seconds() > 0.20) { shootState = 7; shootTimer.reset(); }
                    break;

                case 7:
                    shootStatus = "Cycle complete";
                    launchgate.setPosition(LAUNCHGATE_DOWN);

                    // Loop the sequence continuously while trigger is held
                    if (shootTimer.seconds() > 0.10) {
                        if (shootTriggerHeld) {
                            shootState = 2;
                            shootTimer.reset();
                        } else {
                            shooting = false;
                            shootState = 0;
                        }
                    }
                    break;

                default:
                    shootState = 2;
                    shootTimer.reset();
                    break;
            }
        }

        // Y toggle (unchanged)
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) shootingconstant = !shootingconstant;
        lastY = yPressed;

        boolean shooterOn = shootingconstant || shooting;

        // =========================================================
        // Target RPM calculation now uses SWM-compensated distance
        // (your existing regression logic preserved)
        // =========================================================
        double calculatedTargetRPM;
        {
            double clampMax = farShootingEnabled ? FAR_SHOOTING_RPM_MAX : NORMAL_SHOOTING_RPM_MAX;

            double farMinIn = FAR_ZONE_CENTER_IN - FAR_ZONE_HALF_WINDOW_IN;
            double farMaxIn = FAR_ZONE_CENTER_IN + FAR_ZONE_HALF_WINDOW_IN;
            boolean inFarWindow = (distanceToGoalInches >= farMinIn && distanceToGoalInches <= farMaxIn);

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
        }

        boolean isLongRange = distanceToGoalFeet >= 6.0;

        double currentP     = isLongRange ? pLong : p;
        double currentI     = isLongRange ? iLong : i;
        double currentD     = isLongRange ? dLong : d;
        double currentF     = isLongRange ? fLong : f;
        double currentKV    = isLongRange ? kVLong : kV;
        double currentKS    = isLongRange ? kSLong : kS;
        double currentIZone = isLongRange ? I_ZONE_LONG : I_ZONE;

        // Hood regression uses SWM-compensated distance (always active)
        double hoodT = (distanceToGoalFeet - HOOD_MIN_DIST_FT) / (HOOD_MAX_DIST_FT - HOOD_MIN_DIST_FT);
        hoodT = Math.max(0.0, Math.min(1.0, hoodT));
        double currentHoodPos = HOOD_MAX_POS + hoodT * (HOOD_MIN_POS - HOOD_MAX_POS);
        currentHoodPos = Math.max(0.0, Math.min(1.0, currentHoodPos));
        hood1.setPosition(currentHoodPos);
        hood2.setPosition(currentHoodPos);

        shooterPID.setPIDF(currentP, currentI, currentD, currentF);
        shooterPID.setIntegrationBounds(-currentIZone, currentIZone);

        double targetTPS = rpmToTicksPerSec(calculatedTargetRPM);

        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double vAvg = 0.5 * (vR + vL);

        double avgVelocityRPM = ticksPerSecToRPM(vAvg);

        double shooterPower;
        if (shooterOn) {
            double pidfOutput = shooterPID.calculate(vAvg, targetTPS);

            double sgn = Math.signum(targetTPS);
            double additionalFF = (Math.abs(targetTPS) > 1e-6) ? (currentKS * sgn + currentKV * targetTPS) : 0.0;

            shooterPower = pidfOutput + additionalFF;

            double overspeedRPM = avgVelocityRPM - calculatedTargetRPM;
            if (overspeedRPM > BRAKE_RPM_THRESHOLD) {
                double brakePower = -Math.min(BRAKE_MAX_POWER, BRAKE_KP * overspeedRPM);
                shooterPower += brakePower;
            }

            if (avgVelocityRPM >= calculatedTargetRPM && shooterPower > 0) {
                shooterPower = Math.min(shooterPower, 0.5);
            }

            shooterPower = Math.max(-BRAKE_MAX_POWER, Math.min(1.0, shooterPower));

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

        // LEFT TRIGGER -> transfer (unchanged)
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.1;
        if (currentLeftTrigger && !lastLeftTrigger && !autoTransfer) {
            autoTransfer = true;
            transferState = 0;
            transferTimer.reset();
        }
        lastLeftTrigger = currentLeftTrigger;

        String transferStatus = "Ready";

        if (autoTransfer) {
            switch (transferState) {
                case 0:
                    transferStatus = "Rear intake + Gate HALF";
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (transferTimer.seconds() > 0.1) { transferState = 1; transferTimer.reset(); }
                    break;

                case 1:
                    transferStatus = "Front intake + Gate DOWN";
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() > 0.1) { transferState = 2; transferTimer.reset(); }
                    break;

                case 2:
                    transferStatus = "Gate HALF";
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_HALF);
                    if (transferTimer.seconds() > 0.1) { transferState = 3; transferTimer.reset(); }
                    break;

                case 3:
                    transferStatus = "Gate DOWN";
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() > 0.15) { transferState = 4; transferTimer.reset(); }
                    break;

                case 4:
                    transferStatus = "Gate FULL UP";
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (transferTimer.seconds() > 0.1) { transferState = 5; transferTimer.reset(); }
                    break;

                case 5:
                    transferStatus = "Gate FULL DOWN";
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    if (transferTimer.seconds() > 0.1) { transferState = 6; transferTimer.reset(); }
                    break;

                case 6:
                    transferStatus = "Gate FULL UP";
                    intakeback.setPower(1.0);
                    intakefront.setPower(1.0);
                    launchgate.setPosition(LAUNCHGATE_FULL_UP);
                    if (transferTimer.seconds() > 0.1) { transferState = 7; transferTimer.reset(); }
                    break;

                case 7:
                    transferStatus = "Gate FULL DOWN + STOP";
                    launchgate.setPosition(LAUNCHGATE_DOWN);
                    intakeback.setPower(0.0);
                    intakefront.setPower(0.0);
                    autoTransfer = false;
                    transferState = 0;
                    break;
            }
        } else {
            if (!shooting) {
                launchgate.setPosition(LAUNCHGATE_DOWN);
            }
        }

        boolean atTargetSpeed = Math.abs(avgVelocityRPM - calculatedTargetRPM) < RPM_TOLERANCE;
        boolean intakeActive  = (!autoTransfer) && (gamepad1.left_bumper || gamepad1.right_bumper);

        double ledColor = LED_GREEN;
        if (shooterOn && !atTargetSpeed) ledColor = LED_RED;
        else if (shooting || autoTransfer) ledColor = LED_BLUE;
        else if (intakeActive) ledColor = LED_YELLOW;
        setLedColor(ledColor);

        telemetryA.addData("x", currentX);
        telemetryA.addData("y", currentY);
        telemetryA.addData("heading (deg)", Math.toDegrees(currentHeading));

        telemetryA.addLine("=== SWM ===");
        telemetryA.addData("vx (in/s)", "%.1f", vx);
        telemetryA.addData("vy (in/s)", "%.1f", vy);
        telemetryA.addData("GoalCompX", "%.1f", goalXComp);
        telemetryA.addData("GoalCompY", "%.1f", goalYComp);
        telemetryA.addData("DistComp (in)", "%.1f", distanceToGoalInches);
        telemetryA.addData("Airtime (s)", "%.3f", swm.airtimeSeconds);

        telemetryA.addLine("=== Shooter ===");
        telemetryA.addData("Far Shooting Enabled", farShootingEnabled ? "ON" : "OFF");
        telemetryA.addData("Target RPM (motor)", "%.0f", calculatedTargetRPM);
        telemetryA.addData("Current RPM (motor)", "%.0f", avgVelocityRPM);
        telemetryA.addData("Hood Pos", "%.3f", currentHoodPos);
        telemetryA.addData("Shoot", shootTriggerHeld ? "TRIGGER HELD" : "TRIGGER UP");
        telemetryA.addData("ShootState", "%d", shootState);
        telemetryA.addData("ShootStatus", shootStatus);
        telemetryA.addData("TransferStatus", transferStatus);

        telemetryA.update();
    }

    // =========================================================
    // SWM helper types + computation
    // =========================================================
    private static class SwmSolution {
        final double goalXComp;
        final double goalYComp;
        final double distanceInchesComp;
        final double airtimeSeconds;

        SwmSolution(double gx, double gy, double distIn, double t) {
            this.goalXComp = gx;
            this.goalYComp = gy;
            this.distanceInchesComp = distIn;
            this.airtimeSeconds = t;
        }
    }

    private SwmSolution computeSwmCompensatedGoal(
            double robotX, double robotY,
            double goalX, double goalY,
            double vxInPerSec, double vyInPerSec,
            boolean farEnabled
    ) {
        double gx = goalX;
        double gy = goalY;

        double finalDist = Math.hypot(gx - robotX, gy - robotY);
        double t = 0.30; // seed

        for (int iter = 0; iter < Math.max(1, AIRTIME_ITERATIONS); iter++) {
            double dx = gx - robotX;
            double dy = gy - robotY;
            double distIn = Math.hypot(dx, dy);
            double distFt = distIn / 12.0;

            // Use your existing RPM setpoint math (motor RPM) as the basis for airtime estimation.
            double motorRpm;
            {
                double clampMax = farEnabled ? FAR_SHOOTING_RPM_MAX : NORMAL_SHOOTING_RPM_MAX;

                double farMinIn = FAR_ZONE_CENTER_IN - FAR_ZONE_HALF_WINDOW_IN;
                double farMaxIn = FAR_ZONE_CENTER_IN + FAR_ZONE_HALF_WINDOW_IN;
                boolean inFarWindow = (distIn >= farMinIn && distIn <= farMaxIn);

                if (farEnabled && inFarWindow) {
                    double baseRpmAtCenter = FAR_SHOOTING_RPM_MAX;
                    double rpmPerInch = RPM_SLOPE / 12.0;
                    motorRpm = baseRpmAtCenter + rpmPerInch * (distIn - FAR_ZONE_CENTER_IN);

                    double farWindowMax = FAR_SHOOTING_RPM_MAX + FAR_ZONE_RPM_HEADROOM;
                    motorRpm = Math.max(600, Math.min(farWindowMax, motorRpm));

                } else if (farEnabled && distFt >= 9.0) {
                    motorRpm = FAR_SHOOTING_RPM_MAX;

                } else {
                    motorRpm = RPM_SLOPE * distFt + RPM_INTERCEPT;
                    motorRpm = Math.max(600, Math.min(clampMax, motorRpm));
                }
            }

            // Convert motor RPM to flywheel RPM (23:20 motor->flywheel)
            double flywheelRpm = motorRpm * MOTOR_TO_FLYWHEEL_RATIO;

            // Simple airtime model (tunable)
            double speedInPerSec = AIRTIME_SPEED_PER_FLYWHEEL_RPM * Math.max(1.0, flywheelRpm);
            t = distIn / Math.max(1e-6, speedInPerSec);
            t = Math.max(AIRTIME_MIN_S, Math.min(AIRTIME_MAX_S, t));

            // Velocity compensation: goal appears shifted opposite robot motion
            gx = goalX - vxInPerSec * t;
            gy = goalY - vyInPerSec * t;

            finalDist = distIn;
        }

        // recompute final distance using final compensated goal
        finalDist = Math.hypot(gx - robotX, gy - robotY);
        return new SwmSolution(gx, gy, finalDist, t);
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
