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

@Autonomous(name = "CloseCompatibleBlue")
public class CloseCompatibleBlue extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    // ===== Existing paths (keep) =====
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    // ===== Appended new paths (as path10..path14) =====
    private PathChain path10, path11, path12, path13, path14;

    // =========================
    // Turret target
    // =========================
    public static double targetX = mx(124.0);
    public static double targetY = 125.0;

    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 140.0;  // deg

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    private static final double TURRET_LIVE_T = 0.95;

    // Shooter
    private static final double BASE_TARGET_RPM = 1070;

    private double desiredHoldRpm = BASE_TARGET_RPM;
    private com.acmerobotics.roadrunner.Action currentHoldAction = null;
    private double currentHoldRpm = Double.NaN;

    private static final double GLOBAL_DECEL = 0.58;

    private static final double SHOOT_T = 0.95;
    private static final double PRELOAD_DELAY_S = 0.5;

    private static double mx(double x) { return 144.0 - x; }

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

        // Servo init
        hood1.setPosition(0.475);
        turret1.setPosition(0.175);
        turret2.setPosition(0.175);
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

        follower.setStartingPose(new Pose(34.942, 134.558, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();

        desiredHoldRpm = BASE_TARGET_RPM;
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
        desiredHoldRpm = BASE_TARGET_RPM;
        updateShooterHold(desiredHoldRpm);
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
        // Existing 1..9 (UNCHANGED)
        // ======================

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(34.942, 134.558),
                        new Pose(54.086, 87.900)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
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
                        new Pose(54.086, 87.900),
                        new Pose(37.774, 88.596),
                        new Pose(16.0, 75.0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(16.0, 75.0),
                        new Pose(55.0, 88.114)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.0, 88.114),
                        new Pose(54.084, 56.723),
                        new Pose(9.830, 56.0)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(9.830, 56.0),
                        new Pose(16.361, 60.644),
                        new Pose(16.0, 65.879)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(16.0, 65.879),
                        new Pose(54.393, 87.594)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(54.393, 87.594),
                        new Pose(71.219, 30.869),
                        new Pose(14.319, 35.656)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(14.319, 35.656),
                        new Pose(18.737, 50.521),
                        new Pose(17.453, 70.321)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(17.453, 70.321),
                        new Pose(54.369, 87.600)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        // ======================
        // Appended NEW 5 paths (path10..path14)
        // Mapping:
        //   New Path7  -> path10 (DRIVE)
        //   New Path8  -> path11 (DRIVE)
        //   New Path9  -> path12 (INTAKE)
        //   New Path10 -> path13 (SHOOT)
        //   New Path11 -> path14 (PARK)
        // ======================

        // path10 (drive): (54.294,88.117) -> (16.764,63.189), constant heading 180
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(54.294, 88.117),
                        new Pose(16.764, 63.189)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // path11 (drive): (16.764,63.189) -> (29.508,63.328), linear 180->180
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(16.764, 63.189),
                        new Pose(29.508, 63.328)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // path12 (intake): curve (29.508,63.328)->(30.900,45.931)->(10.641,35.254)
        path12 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(29.508, 63.328),
                        new Pose(30.900, 45.931),
                        new Pose(10.641, 35.254)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // path13 (shoot): line (10.641,35.254)->(54.346,88.208), tangent, reversed
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.641, 35.254),
                        new Pose(54.346, 88.208)
                ))
                .setConstantHeadingInterpolation(180)
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        // path14 (park): line (54.346,88.208)->(45.194,76.606), tangent
        path14 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(54.346, 88.208),
                        new Pose(45.194, 76.606)
                ))
                .setConstantHeadingInterpolation(180)
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        final double WAIT_AFTER_SHOOT = 1.2;

        // Existing 1..9 order (unchanged)
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
        tasks.add(new PathChainTask(path10, 0.0));              // drive
        tasks.add(new PathChainTask(path11, 0.0));              // drive
        tasks.add(new PathChainTask(path12, 0.0));              // intake
        tasks.add(new PathChainTask(path13, 2.0)); // shoot
        tasks.add(new PathChainTask(path14, 0.0));              // park
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
    // Turret aiming
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
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedDeg / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedDeg) / turretMaxAngle) * servoRange;
        }

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
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

        // Predict end pose for shoot paths
        if (active == path1)  return new Pose(54.086, 87.900, Math.toRadians(180));
        if (active == path3)  return new Pose(55.0, 88.114, Math.toRadians(180));
        if (active == path6)  return new Pose(54.393, 87.594, Math.toRadians(180));
        if (active == path9)  return new Pose(54.369, 87.600, Math.toRadians(180));
        if (active == path13) return new Pose(54.346, 88.208, Math.toRadians(180));

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

