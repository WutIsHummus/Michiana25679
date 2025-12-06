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

@Config
@TeleOp(name = "Sunnyred")
public class SunnyRed extends OpMode {
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

    // === Red-Side Field Constants (unmirrored) ===
    public static double targetX = 128.0;
    public static double targetY = 125.0;

    // Low-power shooter mode (Right Trigger)
    public static double LOW_POWER_RPM = 800.0;
    public static double LOW_POWER_TRIGGER_THRESHOLD = 0.1;

    // Goal zone coordinates (red side)
    public static double goalZoneX = 116.0;
    public static double goalZoneY = 116.0;

    // Snap-to-pose (red side)
    public static double SNAP_X = 101.3293;
    public static double SNAP_Y = 123.7003;
    public static double SNAP_HEADING_DEG = 359.0;

    public static double turretTrimDeg = 0.0;
    public static double TRIM_STEP_DEG = 3.0;

    // Height measurements
    public static double aprilTagHeight = 30.0; // AprilTag height in inches
    public static double limelightHeight = 13.5; // Limelight height in inches
    public static double heightDifference = aprilTagHeight - limelightHeight;
    public static double limelightMountAngle = 19.0; // degrees

    // Turret servo constants
    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition = 0.15;
    public static double turretRightPosition = 0.85;
    public static double turretMaxAngle = 147;

    // Shooter PIDF Constants
    public static double TICKS_PER_REV = 28.0;
    public static double GEAR_RATIO = 1.0;

    // Short range PIDF (< 6 feet)
    public static double p = 0.0015;
    public static double i = 0.0;
    public static double d = 0.0000;
    public static double f = 0.0009;
    public static double kV = 0.0008;
    public static double kS = 0.0;
    public static double I_ZONE = 250.0;
    public static double hood1Position = 0.54;

    // Long range PIDF (>= 6 feet)
    public static double pLong = 0.0015;
    public static double iLong = 0.0;
    public static double dLong = 0;
    public static double fLong = 0.00089;
    public static double kVLong = 0.0008;
    public static double kSLong = 0.0;
    public static double I_ZONE_LONG = 250.0;
    public static double hood1PositionLong = 0.45;

    private boolean lastB = false;

    // Linear regression for RPM calculation: RPM = 80 * (feet_from_goal) + 1050
    private static final double RPM_SLOPE = 80.0;
    private static final double RPM_INTERCEPT = 1050.0;

    // Far shooting RPM cap (for distances >= 9 feet)
    public static double FAR_SHOOTING_RPM_MAX = 1800;

    // NEW: normal (non-far) max RPM cap
    public static double NORMAL_SHOOTING_RPM_MAX = 1600.0;

    // Far-shooting toggle state (D-pad UP)
    private boolean farShootingEnabled = false;
    private boolean lastDpadUp = false;

    // Auto-shoot state machine
    private boolean lastA = false;
    private boolean lastLeftTrigger = false;
    private boolean shooting = false;
    private boolean autoTransfer = false;
    private int shootState = 0;
    private int transferState = 0;
    private ElapsedTime shootTimer;
    private ElapsedTime transferTimer;
    // far-shot single-fire state
    private boolean farSingleShot = false;
    private ElapsedTime farShotTimer;

    // RPM tolerance for "at target" detection
    public static double RPM_TOLERANCE = 100.0;
    private boolean shootingconstant = true;
    private boolean lastY = false;

    // --- Hood regression (distance-based) ---
    public static double HOOD_MIN_POS = 0.45;
    public static double HOOD_MAX_POS = 0.54;
    public static double HOOD_MIN_DIST_FT = -1;
    public static double HOOD_MAX_DIST_FT = 7.0;

    // --- Turret backlash compensation ---
    public static double TURRET1_BACKLASH_OFFSET = 0.021;

    // Voltage compensation
    private static final double NOMINAL_VOLTAGE = 12.0;

    // edge detectors
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

    @Override
    public void init() {
        // Follower + pose restore
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

        // We are NOT using Pedro's built-in teleop drive here
        // follower.startTeleopDrive();

        // Drive motors
        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        bl = hardwareMap.get(DcMotorEx.class, "backleft");
        br = hardwareMap.get(DcMotorEx.class, "backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // Turret
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        // Shooter hardware
        intakefront = hardwareMap.get(DcMotorEx.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotorEx.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");

        reargate   = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        hood1      = hardwareMap.get(Servo.class, "hood 1");

        shootl.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeback.setDirection(DcMotorSimple.Direction.REVERSE);
        intakefront.setDirection(DcMotorSimple.Direction.REVERSE);

        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");

        // Motor modes
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        shooterPID = new PIDFController(p, i, d, f);
        shooterPID.setIntegrationBounds(-I_ZONE, I_ZONE);

        shootTimer    = new ElapsedTime();
        transferTimer = new ElapsedTime();
        farShotTimer  = new ElapsedTime();

        launchgate.setPosition(0.5);
        hood1.setPosition(hood1Position);
        setLedColor(LED_GREEN);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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

        telemetryA.addLine("Sunny Red TeleOp - Localization + Shooter");
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

        // Low-power mode (RT)
        boolean lowPowerMode = gamepad1.right_trigger > LOW_POWER_TRIGGER_THRESHOLD;

        if (dpadRight && !lastDpadRight) {
            turretTrimDeg += TRIM_STEP_DEG;
        }
        if (dpadLeft && !lastDpadLeft) {
            turretTrimDeg -= TRIM_STEP_DEG;
        }
        lastDpadLeft  = dpadLeft;
        lastDpadRight = dpadRight;

        // FAR SHOOTING TOGGLE: D-pad UP
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp) {
            farShootingEnabled = !farShootingEnabled;
        }
        lastDpadUp = dpadUp;

        // Update localization only; drive is fully manual
        follower.update();

        // Snap pose on X
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastX) {
            double snapHeadingRad = Math.toRadians(SNAP_HEADING_DEG);
            follower.setPose(new Pose(SNAP_X, SNAP_Y, snapHeadingRad));
        }
        lastX = xPressed;

        // ================== MANUAL MECANUM DRIVE ==================
        double y  = -gamepad1.left_stick_y;      // forward
        double x  =  gamepad1.left_stick_x * 1.1; // strafe
        double rx =  gamepad1.right_stick_x;     // rotate

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        // ================== END MANUAL DRIVE ==================

        // Pose
        Pose currentPose = follower.getPose();
        double currentX  = currentPose.getX();
        double currentY  = currentPose.getY();
        double currentHeading = currentPose.getHeading();

        // Turret targeting
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

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

        // Intake bumpers
        if (gamepad1.left_bumper) {
            intakeback.setPower(1.0);
        } else if (!autoTransfer && !farSingleShot) {
            intakeback.setPower(0);
        }

        if (gamepad1.right_bumper) {
            intakefront.setPower(1.0);
        } else if (!autoTransfer && !farSingleShot) {
            intakefront.setPower(0);
        }

        // Distance for timing / near vs far
        double deltaGoalXForTiming = goalZoneX - currentX;
        double deltaGoalYForTiming = goalZoneY - currentY;
        double distanceFeetForTiming = Math.sqrt(
                deltaGoalXForTiming * deltaGoalXForTiming +
                        deltaGoalYForTiming * deltaGoalYForTiming
        ) / 12.0;

        boolean isFarForShots = distanceFeetForTiming >= 6.0;
        double shotTimeScale = isFarForShots ? 16.0 : 1.0;

        boolean isFarForTransfer = distanceFeetForTiming >= 18.0;  // left as-is

        // A button auto-shoot
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

        // Y toggles shooterconstant
        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {
            shootingconstant = !shootingconstant;
        }
        lastY = yPressed;

        boolean shooterOn = shootingconstant || shooting;

        // Distance to goal zone for RPM / hood
        double deltaGoalX = goalZoneX - currentX;
        double deltaGoalY = goalZoneY - currentY;
        double distanceToGoalInches = Math.sqrt(deltaGoalX * deltaGoalX + deltaGoalY * deltaGoalY);
        double distanceToGoalFeet = distanceToGoalInches / 12.0;

        // === TARGET RPM CALCULATION WITH FAR TOGGLE ===
        double calculatedTargetRPM;

        if (lowPowerMode) {
            calculatedTargetRPM = LOW_POWER_RPM;
        } else {
            double clampMax = farShootingEnabled ? FAR_SHOOTING_RPM_MAX : NORMAL_SHOOTING_RPM_MAX;

            if (farShootingEnabled && distanceToGoalFeet >= 9.0) {
                calculatedTargetRPM = FAR_SHOOTING_RPM_MAX;
            } else {
                calculatedTargetRPM = RPM_SLOPE * distanceToGoalFeet + RPM_INTERCEPT;
                calculatedTargetRPM = Math.max(1150.0, Math.min(clampMax, calculatedTargetRPM));
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

        // Hood regression
        double hoodT = (distanceToGoalFeet - HOOD_MIN_DIST_FT) / (HOOD_MAX_DIST_FT - HOOD_MIN_DIST_FT);
        hoodT = Math.max(0.0, Math.min(1.0, hoodT));
        double currentHoodPos = HOOD_MAX_POS + hoodT * (HOOD_MIN_POS - HOOD_MAX_POS);
        currentHoodPos = Math.max(0.0, Math.min(1.0, currentHoodPos));
        hood1.setPosition(currentHoodPos);

        shooterPID.setPIDF(currentP, currentI, currentD, currentF);
        shooterPID.setIntegrationBounds(-currentIZone, currentIZone);

        double targetTPS = rpmToTicksPerSec(calculatedTargetRPM);

        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double vAvg = 0.5 * (vR + vL);

        double shootrVelocityRPM = ticksPerSecToRPM(vR);
        double shootlVelocityRPM = ticksPerSecToRPM(vL);
        double avgVelocityRPM    = ticksPerSecToRPM(vAvg);

        double shooterPower = 0;
        double pidfOutput = 0;
        double additionalFF = 0;

        if (shooterOn) {
            pidfOutput = shooterPID.calculate(vAvg, targetTPS);

            double sgn = Math.signum(targetTPS);
            additionalFF = (Math.abs(targetTPS) > 1e-6) ? (currentKS * sgn + currentKV * targetTPS) : 0.0;

            shooterPower = pidfOutput + additionalFF;

            if (avgVelocityRPM >= calculatedTargetRPM && shooterPower > 0) {
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

        // LEFT TRIGGER
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.1;

        if (currentLeftTrigger && !lastLeftTrigger) {
            if (isFarForTransfer) {
                farSingleShot = true;
                farShotTimer.reset();
                autoTransfer = false;
                transferState = 0;
            } else if (!autoTransfer) {
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
                intakeback.setPower(1);
                intakefront.setPower(1);
            } else if (t < 0.15) {
                launchgate.setPosition(0.5);
                intakeback.setPower(1);
                intakefront.setPower(1);
            } else {
                launchgate.setPosition(0.5);
                intakeback.setPower(0);
                intakefront.setPower(0);
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

        // LED state
        boolean atTargetSpeed = Math.abs(avgVelocityRPM - calculatedTargetRPM) < RPM_TOLERANCE;
        boolean intakeActive  = gamepad1.left_bumper || gamepad1.right_bumper;

        double ledColor = LED_GREEN;

        if (shooterOn && !atTargetSpeed) {
            ledColor = LED_RED;
        } else if (shooting || autoTransfer) {
            ledColor = LED_BLUE;
        } else if (intakeActive) {
            ledColor = LED_YELLOW;
        }
        setLedColor(ledColor);

        // Telemetry
        telemetryA.addData("x", currentX);
        telemetryA.addData("y", currentY);
        telemetryA.addData("heading (deg)", Math.toDegrees(currentHeading));
        telemetryA.addData("", "");

        telemetryA.addLine("=== AIMING ===");
        telemetryA.addData("Turret Target", "(%.1f, %.1f)", targetX, targetY);
        telemetryA.addData("Distance to Target", "%.2f inches", distance);
        telemetryA.addData("Turret Angle", "%.2f degrees", turretAngleDegrees);
        telemetryA.addData("Turret Servo Position", "%.3f", servoPosition);
        telemetryA.addData("Turret Trim (deg)", "%.1f", turretTrimDeg);
        telemetryA.addData("Far Shooting Enabled", farShootingEnabled ? "ON (D-pad UP)" : "OFF");
        if (turretAngleDegrees < -turretMaxAngle || turretAngleDegrees > turretMaxAngle) {
            telemetryA.addData("WARNING", "Target out of turret range!");
        }
        telemetryA.addData("Low Power Mode", lowPowerMode ? "RT: 800 RPM" : "OFF");

        telemetryA.addData("", "");
        telemetryA.addLine("=== Shooter Status ===");
        telemetryA.addData("Shooter", shooterOn ? "RUNNING" : "STOPPED");
        telemetryA.addData("At Target Speed", atTargetSpeed ? "✓ YES" : "NO");
        if (shooting) {
            telemetryA.addData("Auto-Shoot", shootStatus);
            telemetryA.addData("State", shootState);
        }
        if (autoTransfer) {
            telemetryA.addData("Auto-Transfer", transferStatus);
            telemetryA.addData("Transfer State", transferState);
        }

        telemetryA.addData("Distance Range",
                isLongRange ? "LONG (≥6ft)" : "SHORT (<6ft)");
        telemetryA.addData("Goal Zone (RPM calc)", "(%.1f, %.1f)", goalZoneX, goalZoneY);
        telemetryA.addData("Distance to Goal", "%.2f in (%.2f ft)", distanceToGoalInches, distanceToGoalFeet);
        telemetryA.addData("Calculated Target RPM", "%.0f", calculatedTargetRPM);
        telemetryA.addData("Current RPM", "%.0f", avgVelocityRPM);
        telemetryA.addData("Right Motor RPM", "%.0f", shootrVelocityRPM);
        telemetryA.addData("Left Motor RPM", "%.0f", shootlVelocityRPM);
        telemetryA.addData("Error (RPM)", "%.0f", calculatedTargetRPM - avgVelocityRPM);

        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double compensationFactor = NOMINAL_VOLTAGE / currentVoltage;
        telemetryA.addData("Battery Voltage", "%.2f V", currentVoltage);
        telemetryA.addData("Voltage Comp", "×%.3f", compensationFactor);
        telemetryA.addData("Shooter Power (calc)", "%.3f", shooterPower);

        if (shooterOn) {
            double compensatedPower = shooterPower * compensationFactor;
            telemetryA.addData("Actual Motor Power", "%.3f", compensatedPower);
            telemetryA.addData("PIDF Output", "%.4f", pidfOutput);
            telemetryA.addData("Additional FF", "%.4f", additionalFF);
        }

        telemetryA.addData("Front Intake", gamepad1.right_bumper ? "RUNNING" : "STOPPED");
        telemetryA.addData("Back Intake", gamepad1.left_bumper ? "RUNNING" : "STOPPED");
        telemetryA.addData("Launch Gate", (autoTransfer || farSingleShot) ? "AUTO" : "MANUAL/IDLE");
        telemetryA.addData("Hood Position", "%.2f", currentHoodPos);
        telemetryA.addData("Snap Button (X)", xPressed ? "pressed" : "idle");

        // Limelight telemetry
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetryA.addData("", "");
                telemetryA.addLine("=== Limelight Data ===");
                telemetryA.addData("tx (degrees)", "%.2f", result.getTx());
                telemetryA.addData("ty (degrees)", "%.2f", result.getTy());

                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (!fiducialResults.isEmpty()) {
                    telemetryA.addData("AprilTags Detected", fiducialResults.size());
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        double tx = fr.getTargetXDegrees();
                        double ty = fr.getTargetYDegrees();

                        telemetryA.addData("  Tag ID", "%d", fr.getFiducialId());
                        telemetryA.addData("    X (deg)", "%.2f", tx);
                        telemetryA.addData("    Y (deg)", "%.2f", ty);

                        double angleToTarget = limelightMountAngle + ty;

                        if (Math.abs(angleToTarget) > 0.5) {
                            double horizontalDistance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
                            double diagonalDistance = Math.sqrt(
                                    horizontalDistance * horizontalDistance +
                                            heightDifference * heightDifference
                            );
                            double x_inches = horizontalDistance * Math.tan(Math.toRadians(tx));

                            telemetryA.addData("    Angle to Target", "%.2f deg", angleToTarget);
                            telemetryA.addData("    Horizontal Dist", "%.2f in", horizontalDistance);
                            telemetryA.addData("    Diagonal Dist", "%.2f in", diagonalDistance);
                            telemetryA.addData("    X Offset", "%.2f in", x_inches);
                            telemetryA.addData("    Height Diff", "%.2f in", heightDifference);
                        } else {
                            telemetryA.addData("    Distance", "Invalid angle");
                        }

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
                telemetryA.addData("", "");
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
