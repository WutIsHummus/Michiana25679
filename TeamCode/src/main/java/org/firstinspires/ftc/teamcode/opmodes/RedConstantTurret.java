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

@Autonomous(name = "1 - RedClose18StartFar")
@Disabled
public class RedConstantTurret extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;
    private PathChain path12far, leave;

    // =========================
    // Turret target (RED) - mirrored from BLUE
    // BLUE: targetX = 144 - 125 = 19
    // RED : targetX = 144 - 19 = 125
    // =========================
    public static double targetX = 122.0;
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
    private static final double TURRET_LIVE_T = 0.98;

    // Shooter setpoints
    private static final double BASE_TARGET_RPM   = 1100;
    private static final double LATE_TARGET_RPM   = 1320.0;

    // Boost logic
    private static final double SHOOT_RPM_BOOST = 1700.0;
    private static final double BOOST_EXIT_RPM  = 1100.0;
    private static final double BOOST_REARM_RPM = 850.0;

    private boolean shooterBoostActive = true;
    private boolean afterPath10 = false; // becomes true via callback at start of path10

    // Deceleration settings
    private static final double GLOBAL_DECEL = 0.56;
    private static final double PATH12FAR_DECEL = 0.45;

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
        turret1.setPosition(0.875);
        turret2.setPosition(0.875);
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

        // Start pose mirrored from BLUE (56, 8, 270):
        // X' = 144 - 56 = 88
        // h' = PI - 270deg = -90deg = 270deg (normalized)
        follower.setStartingPose(new Pose(88.0, 8.0, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // Turret aiming (predictive until t>=0.98 on shoot paths)
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

        // Mirroring used (done manually for every point):
        // X' = 144 - X ; Y unchanged
        // Heading mirror (when explicitly set): h' = PI - h  (normalized)

        // PATH 1 (shoot path) - shoot at 0.95
        // BLUE: (56.000, 8.000) -> (56.579, 87.421)
        // RED : (88.000, 8.000) -> (87.421, 87.421)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88.000, 8.000),
                        new Pose(87.421, 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.95, () -> run(actions.launch3preload()))
                .build();

        // PATH 2 (intake)
        // BLUE: (56.579, 87.421) -> (14, 80)
        // RED : (87.421, 87.421) -> (130, 80)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(87.421, 87.421),
                        new Pose(124.0, 80.0)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 3 (shoot path)
        // BLUE: (20, 82) -> (56.792, 87.421)
        // RED : (124, 82) -> (87.208, 87.421)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(124.0, 80.0),
                        new Pose(87.208, 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.85, () -> run(actions.launch3faster()))
                .build();

        // PATH 4 (intake)
        // BLUE curve: (56.792,87.634)->(54.665,29)->(14,28)
        // RED  curve: (87.208,87.634)->(89.335,29)->(130,28)
        // BLUE heading 230->180 becomes RED heading 310->0
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.208, 87.634),
                        new Pose(89.335, 29.0),
                        new Pose(126.0, 28.0)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 5 (shoot path)
        // BLUE curve: (10,25)->(54.665,35.734)->(56.579,87.634)
        // RED  curve: (134,25)->(89.335,35.734)->(87.421,87.634)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(126.0, 28.0),
                        new Pose(89.335, 35.734),
                        new Pose(87.421, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.85, () -> run(actions.launch3faster()))
                .build();

        // PATH 6 (intake)
        // BLUE curve: (56.792,87.421)->(50,48)->(10,52)
        // RED  curve: (87.208,87.421)->(94,48)->(134,52)
        // BLUE heading 230->165 becomes RED heading 310->15
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.208, 87.421),
                        new Pose(94.0, 48.0),
                        new Pose(134, 54)))
                .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(-345))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 7 (shoot path)
        // BLUE curve: (14,52)->(24,50)->(56.792,87.634)
        // RED  curve: (130,52)->(120,50)->(87.208,87.634)
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(134, 52.0),
                        new Pose(120.0, 50.0),
                        new Pose(87.208, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.95, () -> run(actions.launch3faster()))
                .build();

        // PATH 8 (intake)
        // BLUE: (53.176,89.335)->(13,62.109)->(9,51.261)->(9,8)
        // RED : (90.824,89.335)->(131,62.109)->(135,51.261)->(135,8)
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.208, 87.634),
                        new Pose(120, 70.0),
                        new Pose(135.0, 51.261),
                        new Pose(135.0, 8.0)
                ))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 9 (shoot path)
        // BLUE: (11.486,11.273)->(56.579,87.634)
        // RED : (132.514,11.273)->(87.421,87.634)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(135.0, 8.0),
                        new Pose(87.421, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.95, () -> run(actions.launch3faster()))
                .build();

        // PATH 10 (intake) - at t=0, latch RPM change flag
        // BLUE: (56.579,87.634)->(70,10)->(11.061,8.721)
        // RED : (87.421,87.634)->(74,10)->(132.939,8.721)
        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.421, 87.634),
                        new Pose(74.0, 10.0),
                        new Pose(132.939, 8.721)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> {
                    afterPath10 = true;
                    run(actions.startIntake());
                })
                .build();

        // PATH 12FAR (shoot path)
        // BLUE: (11.061,8.721)->(57,13) heading 180
        // RED : (132.939,8.721)->(87,13) heading 0
        path12far = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(132.939, 8.721),
                        new Pose(87.0, 13.0)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration(PATH12FAR_DECEL)
                .addParametricCallback(0.95, () -> run(actions.launch3faster()))
                .build();

        // LEAVE
        // BLUE: (57,13)->(12,8)
        // RED : (87,13)->(132,8)
        leave = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(87.0, 13.0),
                        new Pose(132.0, 8.0)))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        final double WAIT_AFTER_SHOOT = 0.8;

        tasks.add(new PathChainTask(path1, 0.8));
        addPath(path2, 0.0);

        tasks.add(new PathChainTask(path3, WAIT_AFTER_SHOOT));
        addPath(path4, 0.0);

        tasks.add(new PathChainTask(path5, WAIT_AFTER_SHOOT));
        addPath(path6, 0.0);

        tasks.add(new PathChainTask(path7, WAIT_AFTER_SHOOT));
        addPath(path8, 0.0);

        tasks.add(new PathChainTask(path9, WAIT_AFTER_SHOOT));
        addPath(path10, 0.0);

        //addPath(path11, 0.0);

        tasks.add(new PathChainTask(path12far, WAIT_AFTER_SHOOT));
        addPath(leave, 0.0);
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
    // - For shoot paths: use end-of-path pose until t>=TURRET_LIVE_T, then live
    // - Includes clamp (angle clamp to turretMaxAngle)
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

    // --- Predicted end poses (mirrored) ---
    private Pose poseAtEndOfPath1() {
        // RED line: (88.000,8.000) -> (87.421,87.421), reversed
        double h = headingFromLine(88.000, 8.000, 87.421, 87.421, true);
        return new Pose(87.421, 87.421, h);
    }

    private Pose poseAtEndOfPath3() {
        // RED line: (124,82) -> (87.208,87.421), reversed
        double h = headingFromLine(124.0, 82.0, 87.208, 87.421, true);
        return new Pose(87.208, 87.421, h);
    }

    private Pose poseAtEndOfPath5() {
        // RED curve: last control (89.335,35.734) -> end (87.421,87.634), reversed
        double h = headingFromLine(89.335, 35.734, 87.421, 87.634, true);
        return new Pose(87.421, 87.634, h);
    }

    private Pose poseAtEndOfPath7() {
        // RED curve: last control (120,50) -> end (87.208,87.634), reversed
        double h = headingFromLine(120.0, 50.0, 87.208, 87.634, true);
        return new Pose(87.208, 87.634, h);
    }

    private Pose poseAtEndOfPath9() {
        // RED line: (132.514,11.273) -> (87.421,87.634), reversed
        double h = headingFromLine(132.514, 11.273, 87.421, 87.634, true);
        return new Pose(87.421, 87.634, h);
    }

    private Pose poseAtEndOfPath12far() {
        // RED: constant heading 0
        return new Pose(87.0, 13.0, Math.toRadians(0));
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
