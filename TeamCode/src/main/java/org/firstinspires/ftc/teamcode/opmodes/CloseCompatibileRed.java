package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "CloseCompatibleRed")
public class CloseCompatibileRed extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    // ===== Existing paths (kept, but mirrored in buildPathChains) =====
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    // ===== Appended paths (kept, but mirrored in buildPathChains) =====
    private PathChain path10, path11, path12, path13, path14;

    // =========================
    // MIRROR (BLUE -> RED) across X axis:
    // (x, y, heading) -> (x, 144 - y, -heading)
    // =========================
    private static final double FIELD_SIZE = 144.0;

    private static double my(double y) { return FIELD_SIZE - y; }

    private static double mh(double headingRad) { return normalizeRadians(-headingRad); }

    private static Pose p(double x, double y) { return new Pose(x, my(y)); }

    private static Pose p(double x, double y, double headingRad) { return new Pose(x, my(y), mh(headingRad)); }

    // =========================
    // Turret target (RED) + shift target 3in left
    // Original BLUE used targetX = mx(124) = 20.
    // Mirroring across X axis doesn't change X, so RED targetX base = 124.
    // Shift 3in left => 121.
    // targetY mirrors: 125 -> 19
    // =========================
    public static double targetX = 14.0;
    public static double targetY = 125.0; // 19.0

    // Turret servo constants (FLIP left/right for RED)
    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition   = 0.85; // flipped
    public static double turretRightPosition  = 0.15; // flipped
    public static double turretMaxAngle       = 140.0;

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025; // applied with opposite sign below

    private static final double TURRET_LIVE_T = 0.95;

    // Shooter
    private static final double BASE_TARGET_RPM = 1070;

    private com.acmerobotics.roadrunner.Action currentHoldAction = null;
    private double currentHoldRpm = Double.NaN;

    private static final double GLOBAL_DECEL = 0.58;

    private static final double SHOOT_T = 0.95;
    private static final double PRELOAD_DELAY_S = 0.5;

    @Override
    public void init() {
        super.init();

        pathTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotor.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");

        hood1       = hardwareMap.get(Servo.class, "hood 1");
        turret1     = hardwareMap.get(Servo.class, "turret1");
        turret2     = hardwareMap.get(Servo.class, "turret2");
        reargate    = hardwareMap.get(Servo.class, "reargate");
        launchgate  = hardwareMap.get(Servo.class, "launchgate");

        indexfront  = hardwareMap.get(Servo.class, "indexfront");
        indexback   = hardwareMap.get(Servo.class, "indexback");

        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servo init (same as BLUE baseline)
        hood1.setPosition(0.475);
        turret1.setPosition(0.875);
        turret2.setPosition(0.875);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);

        indexfront.setPosition(RobotActions.INDEX_FRONT_RETRACTED);
        indexback.setPosition(RobotActions.INDEX_BACK_EXTENDED);

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                indexfront, indexback,
                hardwareMap.voltageSensor.iterator().next()
        );

        run(actions.safeindexer());

        // Start pose mirrored:
        // BLUE start (34.942, 134.558, 270deg) -> RED (34.942, 9.442, -270deg)
        follower.setStartingPose(p(34.942, 134.558, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
        updateShooterHold(BASE_TARGET_RPM);
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        updateTurret();
        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // ======================
        // Existing 1..9 (mirrored across X axis)
        // ======================

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(34.942, 134.558),
                        p(54.086, 87.900)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(270)), mh(Math.toRadians(180)))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(PRELOAD_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(54.086, 87.900),
                        p(37.774, 88.596),
                        p(20.0, 75.0)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(180)), mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(20.0, 75.0),
                        p(59.0, 88.114)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(59.0, 88.114),
                        p(57.0, 56.723),
                        p(13.0, 56.0)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(13.0, 56.0),
                        p(19.0, 60.644),
                        p(20.0, 65.879)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(180)), mh(Math.toRadians(270)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(20.0, 65.879),
                        p(59.0, 87.594)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(270)), mh(Math.toRadians(180)))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(59.0, 87.594),
                        p(75.0, 30.869),
                        p(18.0, 35.656)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(18.0, 35.656),
                        p(22.0, 50.521),
                        p(21.0, 70.321)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(180)), mh(Math.toRadians(270)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(21.0, 70.321),
                        p(58.0, 87.600)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(270)), mh(Math.toRadians(180)))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        // ======================
        // Appended 5 (mirrored)
        // ======================

        // path10 (drive)
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(58.0, 88.117),
                        p(20.0, 63.189)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.stopIntake()))
                .build();

        // path11 (drive)
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(20.0, 63.189),
                        p(32.0, 63.328)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(180)), mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.stopIntake()))
                .build();

        // path12 (intake)
        path12 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(32.0, 63.328),
                        p(33.0, 45.931),
                        p(13.0, 35.254)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(180)), mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // path13 (shoot) - matches your generator: tangent + reversed
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(13.0, 35.254),
                        p(58.0, 88.208)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        // path14 (park) - tangent
        path14 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(58.0, 88.208),
                        p(49.0, 76.606)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                 .addParametricCallback(0.0, () -> run(actions.stopIntake()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        final double WAIT_AFTER_SHOOT = 1.2;

        // Existing 1..9 order
        tasks.add(new PathChainTask(path1, 2.0));
        tasks.add(new PathChainTask(path2, 0.0));
        tasks.add(new PathChainTask(path3, WAIT_AFTER_SHOOT));
        tasks.add(new PathChainTask(path4, 0.0));
        tasks.add(new PathChainTask(path5, 1.0));
        tasks.add(new PathChainTask(path6, WAIT_AFTER_SHOOT));
        tasks.add(new PathChainTask(path7, 0.0));
        tasks.add(new PathChainTask(path8, 0.0));
        tasks.add(new PathChainTask(path9, WAIT_AFTER_SHOOT));

        // Append new 5: drive, drive, intake, shoot, park
        tasks.add(new PathChainTask(path10, 0.0));
        tasks.add(new PathChainTask(path11, 0.0));
        tasks.add(new PathChainTask(path12, 0.0));
        tasks.add(new PathChainTask(path13, 2.0));
        tasks.add(new PathChainTask(path14, 0.0));
    }

    @Override
    protected boolean isPathActive() { return follower.isBusy(); }

    @Override
    protected boolean isTurning() { return false; }

    @Override
    protected double getCurrentTValue() { return follower.getCurrentTValue(); }

    @Override
    protected void startPath(PathChainTask task) {
        follower.followPath((PathChain) task.pathChain, true); // holdEnd = true
    }

    @Override
    protected void startTurn(TurnTask task) { }

    // =========================
    // Turret aiming (flipped)
    // =========================
    private void updateTurret() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getAimPoseForTurret();

        double dx = targetX - aimPose.getX();
        double dy = targetY - aimPose.getY();

        double angleToTargetField = Math.atan2(dy, dx);
        double turretAngle = normalizeRadians(angleToTargetField - aimPose.getHeading());

        double turretDeg = Math.toDegrees(turretAngle) + turretTrimDeg;
        double clampedDeg = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretDeg));

        double servoPosition;
        if (clampedDeg >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition; // (note: flipped constants)
            servoPosition = turretCenterPosition + (clampedDeg / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;  // (note: flipped constants)
            servoPosition = turretCenterPosition - (Math.abs(clampedDeg) / turretMaxAngle) * servoRange;
        }

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // Backlash offset flipped (subtract instead of add)
        double turret1Pos = servoPosition - TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);
    }

    private Pose getAimPoseForTurret() {
        Pose live = follower.getPose();

        if (taskPhase != 0) return live;

        double t = follower.getCurrentTValue();
        if (t >= TURRET_LIVE_T) return live;

        PathChain active = getActivePathIfAny();
        if (active == null) return live;

        // Predicted end poses for shoot paths (mirror Y + flip heading)
        if (active == path1)  return p(54.086, 87.900, Math.toRadians(180));
        if (active == path3)  return p(55.0,   88.114, Math.toRadians(180));
        if (active == path6)  return p(54.393, 87.594, Math.toRadians(180));
        if (active == path9)  return p(54.369, 87.600, Math.toRadians(180));
        if (active == path13) return p(54.346, 88.208, Math.toRadians(180));

        return live;
    }

    private PathChain getActivePathIfAny() {
        if (currentTaskIndex < 0 || currentTaskIndex >= tasks.size()) return null;
        Object taskObj = tasks.get(currentTaskIndex);
        if (!(taskObj instanceof PathChainTask)) return null;
        PathChainTask pct = (PathChainTask) taskObj;
        if (!(pct.pathChain instanceof PathChain)) return null;
        return (PathChain) pct.pathChain;
    }

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // =========================
    // Shooter hold
    // =========================
    private void updateShooterHold(double targetRpm) {
        if (actions == null) return;
        boolean holdMissing = currentHoldAction == null || !runningActions.contains(currentHoldAction);
        if (holdMissing || Math.abs(targetRpm - currentHoldRpm) > 1e-6) {
            if (currentHoldAction != null) stop(currentHoldAction);
            currentHoldRpm = targetRpm;
            currentHoldAction = actions.holdShooterAtRPMclose(targetRpm, 9999);
            run(currentHoldAction);
        }
    }
}
