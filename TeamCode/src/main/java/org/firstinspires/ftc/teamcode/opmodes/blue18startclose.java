package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "2 - blue18startclose")
@Disabled
public class blue18startclose extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path65, path8, path9, path10, path11;

    // =========================
    // Turret target (BLUE)
    // =========================
    public static double targetX = 144.0 - 122; // 19.0 (keep your “18ball style” offset logic)
    public static double targetY = 125.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 140.0;  // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    // Live tracking after this (predictive aim before this on SHOOT paths)
    private static final double TURRET_LIVE_T = 0.98;

    // Shooter base / late setpoints (same behavior as your Blue18 / 18ball-style request)
    private static final double BASE_TARGET_RPM = 1090;
    private static final double LATE_TARGET_RPM = 1090;

    // Boost logic
    private static final double SHOOT_RPM_BOOST = 1700.0;
    private static final double BOOST_EXIT_RPM  = 1090;
    private static final double BOOST_REARM_RPM = 850.0;

    private boolean shooterBoostActive = true;

    // Flip to late RPM at t=0 on Path10 (requested)
    private boolean afterPath10 = false;

    // Deceleration (use same for shoot/nonshoot)
    private static final double GLOBAL_DECEL = 0.47;

    // Shoot callback timing
    private static final double SHOOT_CALLBACK_T = 0.95;

    // Wait after shoot paths (18ball-style)
    private static final double WAIT_AFTER_SHOOT = 0.8;

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

        // Servo init (match your typical safe baseline)
        hood1.setPosition(0.475);
        turret1.setPosition(0.175);
        turret2.setPosition(0.175);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);

        // Indexer baseline + required safeindexer in init
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

        // Start pose = first pose of Path1
        follower.setStartingPose(new Pose(34.671, 134.641, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
        afterPath10 = false;
        shooterBoostActive = true;
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // Predictive turret on SHOOT paths until t>=0.98, then live tracking
        updateTurret();

        // Shooter RPM boost/hold around whichever target is active
        double vR   = shootr.getVelocity();
        double vL   = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avgRpm = 0.5 * (rpmR + rpmL);

        double targetRpm = afterPath10 ? LATE_TARGET_RPM : BASE_TARGET_RPM;

        if (shooterBoostActive) {
            if (avgRpm >= BOOST_EXIT_RPM) shooterBoostActive = false;
        } else {
            if (avgRpm <= BOOST_REARM_RPM) shooterBoostActive = true;
        }

        double requestedRpm = shooterBoostActive ? SHOOT_RPM_BOOST : targetRpm;
        run(actions.holdShooterAtRPMclose(requestedRpm, 30));

        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("afterPath10", afterPath10);
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Mode", shooterBoostActive ? "BOOST" : "HOLD");
        telemetry.addData("Requested RPM", "%.0f", requestedRpm);
        telemetry.addData("RPM avg", "%.0f", avgRpm);
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // =========================
        // Paths exactly as you sent (with 18ball behaviors)
        // =========================

        // PATH 1 (SHOOT) - velocity limit 20, decel 0.55, shoot at 0.95
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(34.671, 134.641),
                        new Pose(55.516, 84.869)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setGlobalDeceleration(GLOBAL_DECEL)
                // NOTE: if your Pedro version uses a different API name, change this line accordingly.
                .addParametricCallback(0.0, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(new SequentialAction(
                        new SleepAction(0.3),
                        actions.launch3faster()
                )))
                .build();

        // PATH 2 (INTAKE)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(55.516, 84.869),
                        new Pose(15, 85.081)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 3 (SHOOT) (reversed)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15, 85.081),
                        new Pose(55.090, 84.869)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(actions.launch3faster()))
                .build();

        // PATH 4 (INTAKE)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.090, 84.869),
                        new Pose(55.516, 35.096),
                        new Pose(8, 35.947)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 5 (SHOOT) (reversed)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(8, 35.947),
                        new Pose(55.728, 35.309),
                        new Pose(55.090, 84.443)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(actions.launch3faster()))
                .build();

        // PATH 6 (INTAKE)
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.090, 84.443),
                        new Pose(58.919, 58.493),
                        new Pose(10.422, 58.706)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 6.5 (INTAKE) (explicitly intake)
        path65 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(10.422, 58.706),
                        new Pose(22.334, 62.109),
                        new Pose(15, 68.916)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 8 (SHOOT)
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15, 68.916),
                        new Pose(55.516, 84.869)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(230))
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, () -> run(actions.stopIntake()))
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(actions.launch3faster()))
                .build();

        // PATH 9 (INTAKE)
        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.516, 84.869),
                        new Pose(11.911, 47.858),
                        new Pose(10.848, 11.273)
                ))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 10 (SHOOT) (reversed) + requested RPM flip callback at t=0
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.848, 11.273),
                        new Pose(55.516, 84.443)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, () -> {
                    afterPath10 = true;          // requested: change variable at callback 0 on 10
                    run(actions.stopIntake());
                })
                .addParametricCallback(SHOOT_CALLBACK_T, () -> run(actions.launch3faster()))
                .build();

        // PATH 11 (INTAKE) (explicitly intake)
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(55.516, 84.443),
                        new Pose(42.753, 71.681)
                ))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // SHOOT paths: PathChainTask + wait after end
        tasks.add(new PathChainTask(path1, 1));

        // INTAKE
        addPath(path2, 0.0);

        // SHOOT
        tasks.add(new PathChainTask(path3, WAIT_AFTER_SHOOT));

        // INTAKE
        addPath(path4, 0.0);

        // SHOOT
        tasks.add(new PathChainTask(path5, WAIT_AFTER_SHOOT));

        // INTAKE
        addPath(path6, 0.0);
        addPath(path65, 0.0);

        // SHOOT
        tasks.add(new PathChainTask(path8, WAIT_AFTER_SHOOT));

        // INTAKE
        addPath(path9, 0.0);

        // SHOOT (RPM flip at t=0 inside path10)
        tasks.add(new PathChainTask(path10, WAIT_AFTER_SHOOT));

        addPath(path9, 0.0);

        // SHOOT (RPM flip at t=0 inside path10)
        tasks.add(new PathChainTask(path10, WAIT_AFTER_SHOOT));

        // INTAKE (explicit)
        addPath(path11, 0.0);
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
    protected void startTurn(TurnTask task) {
        // Not used
    }

    // =========================
    // Minimal predictive turret:
    // - For shoot paths: use end-of-path pose until t>=0.98, then live
    // - Keep clamp (angle clamp to turretMaxAngle)
    // =========================
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

        // Clamp (requested)
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

        // Only while driving
        if (taskPhase != 0) return live;

        double t = follower.getCurrentTValue();
        if (t >= TURRET_LIVE_T) return live;

        PathChain active = getActivePathIfAny();
        if (active == null) return live;

        // Predictive ONLY on shoot paths: 1,3,5,8,10
        if (active == path1)  return poseAtEndOfPath1();
        if (active == path3)  return poseAtEndOfPath3();
        if (active == path5)  return poseAtEndOfPath5();
        if (active == path8)  return poseAtEndOfPath8();
        if (active == path10) return poseAtEndOfPath10();

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

    // --- Predicted end poses (hard-coded from your geometry) ---
    private Pose poseAtEndOfPath1() {
        // Line end heading is explicit via LinearHeadingInterpolation: end = 180
        return new Pose(55.516, 84.869, Math.toRadians(180));
    }

    private Pose poseAtEndOfPath3() {
        // Line: (17.229,85.081)->(55.090,84.869), reversed
        double h = headingFromLine(17.229, 85.081, 55.090, 84.869, true);
        return new Pose(55.090, 84.869, h);
    }

    private Pose poseAtEndOfPath5() {
        // Curve: last control (55.728,35.309) -> end (55.090,84.443), reversed
        double h = headingFromLine(55.728, 35.309, 55.090, 84.443, true);
        return new Pose(55.090, 84.443, h);
    }

    private Pose poseAtEndOfPath8() {
        // Line end heading explicit: 230
        return new Pose(55.516, 84.869, Math.toRadians(230));
    }

    private Pose poseAtEndOfPath10() {
        // Line: (10.848,11.273)->(55.516,84.443), reversed
        double h = headingFromLine(10.848, 11.273, 55.516, 84.443, true);
        return new Pose(55.516, 84.443, h);
    }

    private static double headingFromLine(double x1, double y1, double x2, double y2, boolean reversed) {
        double tangent = Math.atan2(y2 - y1, x2 - x1);
        double h = reversed ? (tangent + Math.PI) : tangent;
        return normalizeRadians(h);
    }

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    @Override
    public void stop() {
        try { PoseStore.save(follower.getPose()); } catch (Exception ignored) {}
        super.stop();
    }
}
