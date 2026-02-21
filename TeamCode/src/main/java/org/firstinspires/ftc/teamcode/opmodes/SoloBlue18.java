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

@Autonomous(name = "SoloBlue18")
public class SoloBlue18 extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    // ===== Paths (new set) =====
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13;

    // =========================
    // Turret target (dynamic: close vs far)
    // =========================
    private static final double FIELD_SIZE_X = 144.0;
    private static double mx(double x) { return FIELD_SIZE_X - x; }

    // Close target baseline (same convention as your other blue autos)
    private static final double TARGET_X_CLOSE = mx(124.0);

    // Far shooting: offset 4" in the OPPOSITE direction from before (so subtract instead of add)
    private static final double TARGET_X_FAR_OFFSET_IN = 2.0;
    private static final double TARGET_X_FAR = TARGET_X_CLOSE - TARGET_X_FAR_OFFSET_IN;

    public static double targetX = TARGET_X_CLOSE;
    public static double targetY = 125.0;

    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 140.0;  // deg

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    private static final double TURRET_LIVE_T = 0.95;

    // =========================
    // Shooter RPM + hood positions
    // =========================
    private static final double CLOSE_RPM = 1070;
    private static final double FAR_RPM   = 1370;

    // Hood: close vs far
    private static final double HOOD_CLOSE = 0.475;
    private static final double HOOD_FAR   = 0.45;   // adjust if your tuned far hood differs

    private com.acmerobotics.roadrunner.Action currentHoldAction = null;
    private double currentHoldRpm = Double.NaN;

    // Timing rules
    private static final double SHOOT_T = 0.95;

    // Preload delay unchanged unless you say otherwise
    private static final double PRELOAD_DELAY_S = 0.5;

    // Increase shooting waits by +0.1s (this is the delay right before launching)
    private static final double SHOOT_EXTRA_DELAY_S = 0.1;
    private static final double SHOOT_DELAY_S = 0.10 + SHOOT_EXTRA_DELAY_S; // was 0.10 -> now 0.20

    // Decel
    private static final double GLOBAL_DECEL = 0.58;

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
        hood1.setPosition(HOOD_CLOSE);
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

        // Start pose = Path1 start pose (heading 270 deg)
        follower.setStartingPose(new Pose(57.489, 9.276, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
        setCloseShootingState();
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
        telemetry.addData("TargetX", targetX);
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // Path1: (57.489,9.276)->(57.914,85.347) constant heading 270
        // Preload shoot here; DO NOT spin intake during path1
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(57.489, 9.276),
                        new Pose(57.914, 85.347)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.5, () -> {
                    run(actions.stopIntake());
                    setCloseShootingState();
                })
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(PRELOAD_DELAY_S + SHOOT_EXTRA_DELAY_S), // increased by +0.1
                        actions.launch3faster()
                )))
                .build();

        // Path2: (57.914,85.347)->(18.117,84.012) constant heading 180 (intake)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(57.914, 85.347),
                        new Pose(20.0, 84.012)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path3: (18.117,84.012)->(58.171,85.065) constant heading 180 (close shoot)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20.0, 84.012),
                        new Pose(58.171, 85.065)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, this::setCloseShootingState)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // Path4: curve (58.171,85.065)->(65.394,28.004)->(10.210,35.521) constant heading 180 (intake)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(58.171, 85.065),
                        new Pose(65.394, 28.004),
                        new Pose(14.0, 35.521)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path5: (10.210,35.521)->(58.058,84.713) constant heading 180 (close shoot)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(14.0, 35.521),
                        new Pose(65.394, 28.004),
                        new Pose(58.058, 84.713)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, this::setCloseShootingState)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // Path6: curve (58.058,84.713)->(64.662,59.770)->(10.003,58.130) constant heading 180 (intake)
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(58.058, 84.713),
                        new Pose(64.662, 59.770),
                        new Pose(14.0, 53.0)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path7: (10.003,58.130)->(16.784,68.028) linear heading 180->270 (intake)
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(14.0, 53.0),
                        new Pose(16.784, 68.028)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path8: (16.784,68.028)->(58.074,85.157) linear heading 270->180 (close shoot)
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(16.784, 68.028),
                        new Pose(58.074, 85.157)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, this::setCloseShootingState)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // Path9: curve (58.074,85.157)->(12.645,62.026)->(9.672,20.263) linear heading 180->270 (intake/drive)
        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(58.074, 85.157),
                        new Pose(12.645, 62.026),
                        new Pose(9.672, 20.263)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path10: (9.672,20.263)->(57.954,84.994) linear heading 270->270 (close shoot)
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(9.672, 20.263),
                        new Pose(57.954, 84.994)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, this::setCloseShootingState)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // Path11: curve (57.954,84.994)->(65.840,11.093)->(10.767,9.981) linear heading 270->180 (intake)
        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(57.954, 84.994),
                        new Pose(65.840, 9.00),
                        new Pose(10.767, 9.981)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path12: (10.767,9.981)->(51.487,14.095) constant heading 180 (FAR shoot)
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.767, 9.981),
                        new Pose(51.487, 14.095)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, this::setFarShootingState)
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // Path13: (51.487,14.095)->(35.254,70.477) constant heading 180 (park / no shoot)
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(51.487, 14.095),
                        new Pose(35.254, 70.477)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        final double WAIT_AFTER_SHOOT = 1.2;

        tasks.add(new PathChainTask(path1, 2.0));
        tasks.add(new PathChainTask(path2, 0.0));
        tasks.add(new PathChainTask(path3, WAIT_AFTER_SHOOT));
        tasks.add(new PathChainTask(path4, 0.0));
        tasks.add(new PathChainTask(path5, WAIT_AFTER_SHOOT));
        tasks.add(new PathChainTask(path6, 0.0));
        tasks.add(new PathChainTask(path7, 0.0));
        tasks.add(new PathChainTask(path8, WAIT_AFTER_SHOOT));
        tasks.add(new PathChainTask(path9, 0.0));
        tasks.add(new PathChainTask(path10, WAIT_AFTER_SHOOT));
        tasks.add(new PathChainTask(path11, 0.0));
        tasks.add(new PathChainTask(path12, WAIT_AFTER_SHOOT));
        tasks.add(new PathChainTask(path13, 0.0));
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

        // Predict end pose for shoot paths only
        if (active == path1)  return new Pose(57.914, 85.347, Math.toRadians(270));
        if (active == path3)  return new Pose(58.171, 85.065, Math.toRadians(180));
        if (active == path5)  return new Pose(58.058, 84.713, Math.toRadians(180));
        if (active == path8)  return new Pose(58.074, 85.157, Math.toRadians(180));
        if (active == path10) return new Pose(57.954, 84.994, Math.toRadians(270));
        if (active == path12) return new Pose(51.487, 14.095, Math.toRadians(180));

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
    // Shooter + hood helpers
    // =========================
    private void setCloseShootingState() {
        targetX = TARGET_X_CLOSE;
        if (hood1 != null) hood1.setPosition(HOOD_CLOSE);
        updateShooterHold(CLOSE_RPM);
        run(actions.stopIntake());
    }

    private void setFarShootingState() {
        targetX = TARGET_X_FAR; // opposite direction: -4"
        if (hood1 != null) hood1.setPosition(HOOD_FAR);
        updateShooterHold(FAR_RPM);
        run(actions.stopIntake());
    }

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
