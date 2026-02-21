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

@Autonomous(name = "SoloFar18Blue")
public class SoloFar18Blue extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    // ===== NEW 13 paths from your Paths class =====
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13;

    // =========================
    // SHOOT RULES
    // - ONLY shoot on shoot paths
    // - Path1 (preload): wait before shooting (your preload wait)
    // - All other shoot paths: standard wait
    // =========================
    private static final double SHOOT_T = 0.95;
    private static final double PRELOAD_WAIT_S = 2.0;     // preload wait before shooting
    private static final double STANDARD_SHOOT_WAIT_S = 0.10;

    // =========================
    // Shooter / hood constants (keep your tuned values)
    // =========================
    private static final double TARGET_RPM = 1370;
    private static final double HOOD_POS   = 0.44;

    private com.acmerobotics.roadrunner.Action currentHoldAction = null;
    private double currentHoldRpm = Double.NaN;

    // =========================
    // Goal / turret (keep your existing scheme)
    // =========================
    public static double FIELD_SIZE_X_IN = 144.0;

    private static double mirrorX(double x) { return FIELD_SIZE_X_IN - x; }

    private static double mirrorHeadingRad(double h) {
        return normalizeRadians(Math.PI - h);
    }

    // Your existing comment said RED targetX=129 -> BLUE=15, keep that logic:
    public static double targetX = mirrorX(132.0);
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

        // Start pose = Path1 start pose from your NEW Paths list
        follower.setStartingPose(new Pose(57.489, 9.064, Math.toRadians(180)));

        buildPathChains();
        buildTaskList();

        // Start holding shooter (always on for this auto)
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

        // =========================
        // Build paths exactly as provided
        // Decide SHOOT vs INTAKE by path type:
        // - "shoot paths" are the ones that end at the shooting pose cluster: (~53.x, 15.x)
        //   Here: Path1, Path3, Path5, Path8, Path10, Path12 are returns-to-shoot
        // - Everything else is intake
        // =========================

        // Path1 (shoot preload): (57.489, 9.064) -> (53.176, 15.740)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(57.489, 9.064),
                        new Pose(53.176, 15.740)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(PRELOAD_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path2 (intake): (53.176, 15.740) -> (10.291, 9.216)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(53.176, 15.740),
                        new Pose(10.291, 9.216)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path3 (shoot): (10.291, 9.216) -> (53.421, 15.582)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.291, 9.216),
                        new Pose(53.421, 15.582)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path4 (intake): curve (53.421, 15.582) -> (48.069, 40.423) -> (10.684, 36.207)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(53.421, 15.582),
                        new Pose(48.069, 40.423),
                        new Pose(10.684, 36.207)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path5 (shoot): (10.684, 36.207) -> (53.685, 15.694)
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.684, 36.207),
                        new Pose(53.685, 15.694)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path6 (intake): curve (53.685, 15.694) -> (59.151, 63.193) -> (9.572, 58.706)
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(53.685, 15.694),
                        new Pose(59.151, 63.193),
                        new Pose(9.572, 58.706)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path7 (intake): curve (9.572, 58.706) -> (19.112, 61.408) -> (16.486, 68.065)
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(9.572, 58.706),
                        new Pose(19.112, 61.408),
                        new Pose(16.486, 68.065)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path8 (shoot): (16.486, 68.065) -> (53.575, 15.592)
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(16.486, 68.065),
                        new Pose(53.575, 15.592)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path9 (intake): (53.575, 15.592) -> (10.012, 9.653)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(53.575, 15.592),
                        new Pose(10.012, 9.653)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path10 (shoot): (10.012, 9.653) -> (53.715, 15.258)
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.012, 9.653),
                        new Pose(53.715, 15.258)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path11 (intake): (53.715, 15.258) -> (11.061, 22.759)
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(53.715, 15.258),
                        new Pose(11.061, 22.759)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path12 (shoot): (11.061, 22.759) -> (53.465, 15.303)
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(11.061, 22.759),
                        new Pose(53.465, 15.303)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(STANDARD_SHOOT_WAIT_S),
                        actions.launch3faster()
                )))
                .build();

        // Path13 (leave/park intake-off): (53.465, 15.303) -> (44.589, 21.232)
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(53.465, 15.303),
                        new Pose(44.589, 21.232)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        final double WAIT_AFTER_SHOOT = 1.2;

        // Run in exact provided order: Path1..Path13
        tasks.add(new PathChainTask(path1, 3.2));                 // preload: wait is inside callback
        tasks.add(new PathChainTask(path2, 0.3));                 // intake
        tasks.add(new PathChainTask(path3, WAIT_AFTER_SHOOT));    // shoot
        tasks.add(new PathChainTask(path4, 0.0));                 // intake
        tasks.add(new PathChainTask(path5, WAIT_AFTER_SHOOT));    // shoot
        tasks.add(new PathChainTask(path6, 0.0));                 // intake
        tasks.add(new PathChainTask(path7, 0.0));                 // intake
        tasks.add(new PathChainTask(path8, WAIT_AFTER_SHOOT));    // shoot
        tasks.add(new PathChainTask(path9, 0.0));                 // intake
        tasks.add(new PathChainTask(path10, WAIT_AFTER_SHOOT));   // shoot
        tasks.add(new PathChainTask(path9, 0.0));                // intake
        tasks.add(new PathChainTask(path10, WAIT_AFTER_SHOOT));   // shoot
        tasks.add(new PathChainTask(path13, 0.0));                // park
    }

    // =========================
    // Shooter hold (simple, constant)
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
    // Turret aiming (predictive -> live at t>=TURRET_LIVE_T)
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

        // Predict for shoot paths only (end poses)
        if (active == path1)  return new Pose(53.176, 15.740, Math.toRadians(180));
        if (active == path3)  return new Pose(53.421, 15.582, Math.toRadians(180));
        if (active == path5)  return new Pose(53.685, 15.694, Math.toRadians(180));
        if (active == path8)  return new Pose(53.575, 15.592, Math.toRadians(180));
        if (active == path10) return new Pose(53.715, 15.258, Math.toRadians(180));
        if (active == path12) return new Pose(53.465, 15.303, Math.toRadians(180));

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

