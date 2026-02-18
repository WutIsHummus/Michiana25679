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

@Autonomous(name = "1 - BlueClose18startfarnewgate")
public class BlueClose18startfarnewgate extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path6_5, path7, path8, path9, path10, path11;
    private PathChain path12far, leave;

    private static double mx(double x) { return 144.0 - x; }

    private static double my(double y) { return y; }

    private static double mh(double headingRad) { return normalizeRadians(Math.PI - headingRad); }

    private static Pose poseX(double x, double y) {
        return new Pose(mx(x), y);
    }

    private static Pose poseX(double x, double y, double headingRad) {
        return new Pose(mx(x), y, mh(headingRad));
    }

    // =========================
    // Turret target (mirrored across X axis)
    // =========================
    public static double targetX = mx(118.0);
    public static double targetY = 125.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 140.0;  // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    // Live tracking after this
    private static final double TURRET_LIVE_T = 0.95;

    // Shooter setpoints
    private static final double BASE_TARGET_RPM   = 1100;
    private static final double LATE_TARGET_RPM   = 1390;

    private boolean afterPath10 = false; // becomes true via callback at start of path10
    private double desiredHoldRpm = BASE_TARGET_RPM;
    private com.acmerobotics.roadrunner.Action currentHoldAction = null;
    private double currentHoldRpm = Double.NaN;

    // Deceleration settings
    private static final double GLOBAL_DECEL = 0.58;
    private static final double PATH12FAR_DECEL = 0.6;

    // =========================
    // New Path 6.5 (from your screenshot)
    // NOTE: these are kept as-is (source constants), but mirrored uses explicit numbers below.
    // =========================
    private static final double P65_END_X  = 15.74002954;
    private static final double P65_END_Y  = 69.341211225;
    private static final double P65_C1_X   = 18.71784296;
    private static final double P65_C1_Y   = 62.3220088;

    // =========================
    // Timing rules (match your working structure)
    // =========================
    private static final double SHOOT_T = 0.95;
    private static final double SHOOT_DELAY_S = 0.1;

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

        // Indexer init to known safe baseline
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

        // Start pose (mirrored across X axis):
        // Red: (84.0, 8.0, -90deg) -> Blue: (84.0, 136.0, 90deg)
        follower.setStartingPose(poseX(84.0, 8.0, Math.toRadians(-90)));

        buildPathChains();
        buildTaskList();

        desiredHoldRpm = BASE_TARGET_RPM;
    }

    @Override
    public void start() {
        super.start();

        // Baseline shooter hold at start (match your working structure)
        //run(actions.holdShooterAtRPMclose(BASE_TARGET_RPM, 9999));

        pathTimer.resetTimer();
        afterPath10 = false;
        desiredHoldRpm = BASE_TARGET_RPM;
        updateShooterHold(desiredHoldRpm);
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // Turret aiming (predictive until t>=0.95 on shoot paths)
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

        // PATH 1 (shoot path) - mirrored across X axis
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseX(84.000, 8.000),
                        poseX(83.421, 84.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.2),
                        actions.launch3faster()
                )))
                .build();

        // PATH 2 (intake)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseX(83.421, 84.421),
                        poseX(118.0, 80.000)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> {
                    desiredHoldRpm = BASE_TARGET_RPM - 20.0;
                    updateShooterHold(desiredHoldRpm);
                    run(actions.startIntake());
                })
                .build();

        // PATH 3 (shoot path)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseX(118.0, 80.000),
                        poseX(83.208, 84.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // PATH 4 (intake)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poseX(83.208, 84.421),
                        poseX(85.335, 29.000),
                        poseX(130.0, 36.00)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 5 (shoot path)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poseX(130.0, 36.00),
                        poseX(85.335, 35.734),
                        poseX(83.421, 84.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.6)
                .setGlobalDeceleration(0.55)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // PATH 6 (intake)
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poseX(83.421, 84.634),
                        poseX(90.000, 48.000),
                        poseX(126.000, 56.000)))
                .setLinearHeadingInterpolation(mh(Math.toRadians(-50)), mh(Math.toRadians(-355)))
                .setTValueConstraint(0.9)
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 6.5 (intake)
        path6_5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poseX(126.000, 56.00),
                        poseX(120, 55),
                        poseX(124, 66.000)
                ))
                .setLinearHeadingInterpolation(mh(Math.toRadians(-355)), mh(Math.toRadians(-90)))
                .setNoDeceleration()
                .setTValueConstraint(0.9)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 7 (shoot path)
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseX(123.000, 66.000),
                        poseX(86.824, 89.335)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.65)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // PATH 8 (intake)
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poseX(86.824, 89.335),
                        poseX(127.000, 30.00),
                        poseX(131.000, 20.000)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(3)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 9 (shoot path)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseX(131.000, 20.000),
                        poseX(83.421, 84.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.65)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // PATH 10 (intake) - latch RPM change
        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        poseX(83.421, 84.634),
                        poseX(70.000, 9.00),
                        poseX(128.00, 8.721)
                ))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> {
                    afterPath10 = true;
                    targetX = mx(118.0);
                    desiredHoldRpm = LATE_TARGET_RPM;
                    updateShooterHold(desiredHoldRpm);
                    run(actions.startIntake());
                })
                .addParametricCallback(0.5, () -> {
                    hood1.setPosition(0.45);
                })
                .build();

        // path12far (shoot)
        path12far = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseX(128.00, 8.721),
                        poseX(83.000, 13.000)))
                .setConstantHeadingInterpolation(mh(Math.toRadians(-360)))
                .setGlobalDeceleration(PATH12FAR_DECEL)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // leave
        leave = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseX(83.000, 13.000),
                        poseX(110.00, 13.00)))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration(0.5)
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        final double WAIT_AFTER_SHOOT = 1.2;

        // IMPORTANT: no addPath() calls; use explicit PathChainTask everywhere (match working structure)

        PathChainTask t1 = new PathChainTask(path1, 1.25);
        tasks.add(t1);

        PathChainTask t2 = new PathChainTask(path2, 0.0);
        tasks.add(t2);

        PathChainTask t3 = new PathChainTask(path3, WAIT_AFTER_SHOOT);
        tasks.add(t3);

        PathChainTask t4 = new PathChainTask(path4, 0.0);
        tasks.add(t4);

        PathChainTask t5 = new PathChainTask(path5, WAIT_AFTER_SHOOT);
        tasks.add(t5);

        PathChainTask t6 = new PathChainTask(path6, 0.0);
        tasks.add(t6);

        PathChainTask t6_5 = new PathChainTask(path6_5, 0.0);
        tasks.add(t6_5);

        PathChainTask t7 = new PathChainTask(path7, WAIT_AFTER_SHOOT);
        tasks.add(t7);

        PathChainTask t8 = new PathChainTask(path8, 0.0);
        tasks.add(t8);

        // After path10 starts, we switch to LATE rpm, but we still keep a hold wait action here too.
        PathChainTask t9 = new PathChainTask(path9, WAIT_AFTER_SHOOT);
        tasks.add(t9);

        PathChainTask t10 = new PathChainTask(path10, 0.0);
        tasks.add(t10);

        PathChainTask t12 = new PathChainTask(path12far, 1.3);
        tasks.add(t12);

        PathChainTask tLeave = new PathChainTask(leave, 0.0);
        tasks.add(tLeave);
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

    private void updateTurret() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getAimPoseForTurret();

        double currentX = aimPose.getX();
        double currentY = aimPose.getY();
        double currentHeading = aimPose.getHeading();

        double dx = targetX - currentX;
        double dy = targetY - currentY;

        double angleToTargetField = Math.atan2(dy, dx);
        double turretAngle = normalizeRadians(angleToTargetField - currentHeading);

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

        if (active == path1) return poseAtEndOfPath1();
        if (active == path3) return poseAtEndOfPath3();
        if (active == path5) return poseAtEndOfPath5();
        if (active == path7) return poseAtEndOfPath7();
        if (active == path9) return poseAtEndOfPath9();
        if (active == path12far) return poseAtEndOfPath12far();

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

    // --- Predicted end poses (mirrored from RedClose18startfarnewgate) ---
    private Pose poseAtEndOfPath1() {
        double tangent = Math.atan2(84.421 - 8.000, 56.579 - 56.000);
        double h = tangent + Math.PI; // reversed
        while (h > Math.PI)  h -= 2.0 * Math.PI;
        while (h < -Math.PI) h += 2.0 * Math.PI;
        h = Math.PI - h;
        while (h > Math.PI)  h -= 2.0 * Math.PI;
        while (h < -Math.PI) h += 2.0 * Math.PI;
        if (h > 0) h -= 2.0 * Math.PI;
        if (Math.abs(h) < 1e-9) h = -2.0 * Math.PI;

        return poseX(83.421, 84.421, h);
    }

    private Pose poseAtEndOfPath3() {
        double tangent = Math.atan2(84.421 - 82.000, 56.792 - 20.000);
        double h = tangent + Math.PI; // reversed
        while (h > Math.PI)  h -= 2.0 * Math.PI;
        while (h < -Math.PI) h += 2.0 * Math.PI;
        h = Math.PI - h;
        while (h > Math.PI)  h -= 2.0 * Math.PI;
        while (h < -Math.PI) h += 2.0 * Math.PI;
        if (h > 0) h -= 2.0 * Math.PI;
        if (Math.abs(h) < 1e-9) h = -2.0 * Math.PI;

        return poseX(83.208, 84.421, h);
    }

    private Pose poseAtEndOfPath5() {
        double tangent = Math.atan2(84.634 - 35.734, 56.579 - 54.665);
        double h = tangent + Math.PI; // reversed
        while (h > Math.PI)  h -= 2.0 * Math.PI;
        while (h < -Math.PI) h += 2.0 * Math.PI;
        h = Math.PI - h;
        while (h > Math.PI)  h -= 2.0 * Math.PI;
        while (h < -Math.PI) h += 2.0 * Math.PI;
        if (h > 0) h -= 2.0 * Math.PI;
        if (Math.abs(h) < 1e-9) h = -2.0 * Math.PI;

        return poseX(83.421, 84.634, h);
    }

    private Pose poseAtEndOfPath7() {
        return poseX(86.824, 89.335, Math.toRadians(-50));
    }

    private Pose poseAtEndOfPath9() {
        double tangent = Math.atan2(84.634 - 20.000, 56.579 - 9.000);
        double h = tangent + Math.PI; // reversed
        while (h > Math.PI)  h -= 2.0 * Math.PI;
        while (h < -Math.PI) h += 2.0 * Math.PI;
        h = Math.PI - h;
        while (h > Math.PI)  h -= 2.0 * Math.PI;
        while (h < -Math.PI) h += 2.0 * Math.PI;
        if (h > 0) h -= 2.0 * Math.PI;
        if (Math.abs(h) < 1e-9) h = -2.0 * Math.PI;

        return poseX(83.421, 84.634, h);
    }

    private Pose poseAtEndOfPath12far() {
        return poseX(83.0, 13.0, Math.toRadians(-360));
    }

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private void updateShooterHold(double targetRpm) {
        if (actions == null) return;
        boolean holdMissing = currentHoldAction == null || !runningActions.contains(currentHoldAction);
        if (holdMissing || Math.abs(targetRpm - currentHoldRpm) > 1e-6) {
            if (currentHoldAction != null) {
                stop(currentHoldAction);
            }
            currentHoldRpm = targetRpm;
            currentHoldAction = actions.holdShooterAtRPMclose(targetRpm, 9999);
            run(currentHoldAction);
        }
    }
}
