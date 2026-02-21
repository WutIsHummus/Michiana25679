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

@Autonomous(name = "SoloFar18Red")
public class SoloFar18Red extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    // ===== RED mirrored paths =====
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13;

    // =========================
    // Mirror + post-shift rules
    // =========================
    // 1) Mirror across X using 144 - x
    // 2) After mirror, shift ALL X poses 4 inches LEFT: x = (144 - x) - 4
    //    => equivalent: x = 140 - x
    // 3) After mirror, shift targetX 4 inches LEFT too.
    // =========================
    private static final double FIELD_X = 144.0;
    private static final double POST_MIRROR_X_SHIFT_LEFT_IN = 4.0;

    private static double mx(double x) {
        return (FIELD_X - x) - POST_MIRROR_X_SHIFT_LEFT_IN; // 140 - x
    }

    private static double mh(double hRad) {
        return normalizeRadians(Math.PI - hRad);
    }

    private static Pose p(double x, double y) {
        return new Pose(mx(x), y);
    }

    private static Pose p(double x, double y, double headingRad) {
        return new Pose(mx(x), y, mh(headingRad));
    }

    // =========================
    // SHOOT RULES (same as Blue)
    // =========================
    private static final double SHOOT_T = 0.95;
    private static final double PRELOAD_WAIT_S = 2.0;
    private static final double STANDARD_SHOOT_WAIT_S = 0.10;

    // Shooter / hood
    private static final double TARGET_RPM = 1370;
    private static final double HOOD_POS   = 0.44;

    private com.acmerobotics.roadrunner.Action currentHoldAction = null;
    private double currentHoldRpm = Double.NaN;

    // =========================
    // Turret target (mirrored + shifted)
    // Blue used targetX = mirrorX(129).
    // Here we do the exact same but with post-mirror shift:
    // targetX = (144 - 129) - 4 = 11
    // =========================
    public static double targetX = 126.00;
    public static double targetY = 125.0;

    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition   = 0.15;
    public static double turretRightPosition  = 0.85;
    public static double turretMaxAngle       = 137;

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    private static final double TURRET_LIVE_T = 0.90;

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
        hood1.setPosition(HOOD_POS);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);

        // Indexer baseline
        indexfront.setPosition(RobotActions.INDEX_FRONT_RETRACTED);
        indexback.setPosition(RobotActions.INDEX_BACK_EXTENDED);

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                indexfront, indexback,
                hardwareMap.voltageSensor.iterator().next()
        );

        // Required
        run(actions.safeindexer());

        // Start pose is mirrored+shifted from the BLUE auto's start pose:
        // BLUE start pose was (57.489, 9.064, 180deg) in your Blue code.
        // Apply mx(): x = 140 - 57.489 = 82.511
        // Heading mirrored: PI - PI = 0
        follower.setStartingPose(p(57.489, 9.064, Math.toRadians(180)));

        buildPathChains();
        buildTaskList();

        updateShooterHold(TARGET_RPM);
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
        updateShooterHold(TARGET_RPM);
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        updateTurretAim();
        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // Original BLUE paths (numbers you provided) are mirrored+shifted here using p(x,y) helpers.

        // Path1 (shoot preload): (57.489, 9.064) -> (53.176, 15.740)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(57.489, 9.064),
                        p(53.176, 15.740)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(PRELOAD_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path2 (intake): (53.176, 15.740) -> (10.291, 9.216)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(53.176, 15.740),
                        p(10.291, 8.0)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path3 (shoot): (10.291, 9.216) -> (53.421, 15.582)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(10.291, 8.0),
                        p(53.421, 15.582)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path4 (intake): curve (53.421, 15.582) -> (48.069, 40.423) -> (10.684, 36.207)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(53.421, 15.582),
                        p(48.069, 40.423),
                        p(10.684, 36.207)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path5 (shoot): (10.684, 36.207) -> (53.685, 15.694)
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(10.684, 36.207),
                        p(53.685, 15.694)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path6 (intake): curve (53.685, 15.694) -> (59.151, 63.193) -> (9.572, 58.706)
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(53.685, 15.694),
                        p(59.151, 63.193),
                        p(9.572, 58.706)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path7 (intake): curve (9.572, 58.706) -> (19.112, 61.408) -> (16.486, 68.065)
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(9.572, 58.706),
                        p(19.112, 61.408),
                        p(16.486, 68.065)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(180)), mh(Math.toRadians(270)))
                .addParametricCallback(0.5, () -> run(actions.startIntake()))
                .build();

        // Path8 (shoot): (16.486, 68.065) -> (53.575, 15.592)
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(16.486, 68.065),
                        p(53.575, 15.592)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(270)), mh(Math.toRadians(180)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path9 (intake): (53.575, 15.592) -> (10.012, 9.653)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(53.575, 15.592),
                        p(10.012, 9.653)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path10 (shoot): (10.012, 9.653) -> (53.715, 15.258)
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(10.012, 9.653),
                        p(53.715, 15.258)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path11 (intake): (53.715, 15.258) -> (11.061, 22.759)
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(53.715, 15.258),
                        p(11.061, 22.759)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path12 (shoot): (11.061, 22.759) -> (53.465, 15.303)
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(11.061, 22.759),
                        p(53.465, 15.303)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path13 (park): (53.465, 15.303) -> (44.589, 21.232)
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(53.465, 15.303),
                        p(44.589, 21.232)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(180)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        final double WAIT_AFTER_SHOOT = 1.2;

        tasks.add(new PathChainTask(path1, 3.2));                 // preload (wait inside callback)
        tasks.add(new PathChainTask(path2, 0.3));                 // intake
        tasks.add(new PathChainTask(path3, WAIT_AFTER_SHOOT));    // shoot
        tasks.add(new PathChainTask(path4, 0.1));                 // intake
        tasks.add(new PathChainTask(path5, WAIT_AFTER_SHOOT));    // shoot
        tasks.add(new PathChainTask(path6, 0.1));                 // intake
        tasks.add(new PathChainTask(path7, 0.1));                 // intake
        tasks.add(new PathChainTask(path8, WAIT_AFTER_SHOOT));    // shoot
        tasks.add(new PathChainTask(path9, 0.1));                 // intake
        tasks.add(new PathChainTask(path10, WAIT_AFTER_SHOOT));   // shoot
        tasks.add(new PathChainTask(path9, 0.3));                // intake
        tasks.add(new PathChainTask(path10, WAIT_AFTER_SHOOT));   // shoot
        tasks.add(new PathChainTask(path13, 0.0));                // park
    }

    // =========================
    // Shooter hold (constant)
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

    @Override
    protected boolean isPathActive() { return follower.isBusy(); }

    @Override
    protected boolean isTurning() { return false; }

    @Override
    protected double getCurrentTValue() { return follower.getCurrentTValue(); }

    @Override
    protected void startPath(PathChainTask task) {
        follower.followPath((PathChain) task.pathChain, true);
    }

    @Override
    protected void startTurn(TurnTask task) { }

    // =========================
    // Turret aiming (predictive -> live)
    // =========================
    private void updateTurretAim() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getAimPoseForTurret();
        setTurretToAimAtPose(aimPose);
    }

    private Pose getAimPoseForTurret() {
        Pose live = follower.getPose();

        if (taskPhase != 0) return live;

        double t = follower.getCurrentTValue();
        if (t >= TURRET_LIVE_T) return live;

        PathChain active = getActivePathIfAny();
        if (active == null) return live;

        // Predict for shoot paths only (use mirrored+shifted endpoints)
        if (active == path1)  return new Pose(mx(53.176), 15.740, mh(Math.toRadians(180)));
        if (active == path3)  return new Pose(mx(53.421), 15.582, mh(Math.toRadians(180)));
        if (active == path5)  return new Pose(mx(53.685), 15.694, mh(Math.toRadians(180)));
        if (active == path8)  return new Pose(mx(53.575), 15.592, mh(Math.toRadians(180)));
        if (active == path10) return new Pose(mx(53.715), 15.258, mh(Math.toRadians(180)));
        if (active == path12) return new Pose(mx(53.465), 15.303, mh(Math.toRadians(180)));

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

    private void setTurretToAimAtPose(Pose pose) {
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();

        double angleToTargetField = Math.atan2(dy, dx);
        double turretAngleRad = normalizeRadians(angleToTargetField - pose.getHeading());
        double turretDeg = Math.toDegrees(turretAngleRad) + turretTrimDeg;

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

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}

