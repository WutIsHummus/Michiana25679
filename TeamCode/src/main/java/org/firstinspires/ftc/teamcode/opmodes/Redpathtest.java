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

@Autonomous(name = "Redpathtest")
public class Redpathtest extends PathChainAutoOpMode {

    private Follower follower;

    // ---------------- Hardware ----------------
    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo launchgate, reargate;
    private Servo turret1, turret2, hood1;

    private RobotActions actions;

    // ---------------- Shooter ----------------
    private static final double SHOOTER_HOLD_RPM = 1130.0;

    // ---------------- Turret ----------------
    public static double targetX = 118.0;
    public static double targetY = 125.0;

    private static final double TURRET_CENTER   = 0.51;
    private static final double TURRET_LEFT     = 0.15;
    private static final double TURRET_RIGHT    = 0.85;
    private static final double TURRET_MAX_DEG  = 140.0;

    private static final double TURRET_BACKLASH = 0.025;
    private static final double TURRET_TRIM_DEG = 0.0;

    // Predictive aiming: live tracking after this t on shoot paths
    private static final double TURRET_LIVE_T = 0.95;

    // ---------------- Paths ----------------
    private PathChain
            path1, path2, path3, path4, path5,
            path6, path6_5, path7, path8,
            path9, path10, path12far, leave;

    @Override
    public void init() {
        super.init();

        follower = Constants.createFollower(hardwareMap);

        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotor.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");

        launchgate  = hardwareMap.get(Servo.class, "launchgate");
        reargate    = hardwareMap.get(Servo.class, "reargate");
        turret1     = hardwareMap.get(Servo.class, "turret1");
        turret2     = hardwareMap.get(Servo.class, "turret2");
        hood1       = hardwareMap.get(Servo.class, "hood 1");

        shootl.setDirection(DcMotor.Direction.REVERSE);
        hood1.setPosition(0.475);

        actions = new RobotActions(
                intakefront, intakeback,
                shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                null, null,
                null
        );

        turret1.setPosition(TURRET_LEFT);
        turret2.setPosition(TURRET_LEFT);

        follower.setStartingPose(new Pose(84.0, 8.0, Math.toRadians(-90)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void loop() {
        super.loop();

        follower.update();
        runTasks();

        // Turret update (predictive)
        updateTurret();
    }

    // =========================================================
    // ======================= PATHS ===========================
    // =========================================================

    @Override
    protected void buildPathChains() {

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(84.000, 8.000),
                        new Pose(83.421, 87.421)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                .addParametricCallback(0.97, () -> run(
                        new SequentialAction(
                                new SleepAction(0.1),
                                actions.launch3faster()
                        )))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(83.421, 87.421),
                        new Pose(123.000, 80.000)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.000, 80.000),
                        new Pose(83.208, 87.421)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                // changed to 0.95 (was 0.85)
                .addParametricCallback(0.97, () -> run(actions.launch3faster()))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.208, 87.421),
                        new Pose(70.0, 29.000),
                        new Pose(130.0, 32.00)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(130.0, 32.00),
                        new Pose(85.335, 35.734),
                        new Pose(83.421, 87.634)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                // changed to 0.95 (was 0.85)
                .addParametricCallback(0.97, () -> run(actions.launch3faster()))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.421, 87.634),
                        new Pose(90.000, 48.000),
                        new Pose(126.000, 52.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(-355))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                .build();

        path6_5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(126.000, 52.000),
                        new Pose(120.000, 55.000),
                        new Pose(124.000, 66.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-355), Math.toRadians(-90))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.000, 66.000),
                        new Pose(86.824, 89.335)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                // already 0.95
                .addParametricCallback(0.97, () -> run(actions.launch3faster()))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(86.824, 89.335),
                        new Pose(127.000, 62.109),
                        new Pose(131.000, 20.000)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(131.000, 20.000),
                        new Pose(83.421, 87.634)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                // already 0.95
                .addParametricCallback(0.97, () -> run(actions.launch3faster()))
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.421, 87.634),
                        new Pose(70.000, 10.000),
                        new Pose(125.000, 8.721)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                .build();

        path12far = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(125.000, 8.721),
                        new Pose(83.000, 13.000)))
                .setConstantHeadingInterpolation(Math.toRadians(-360))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                // changed to 0.95 (was 0.85)
                .addParametricCallback(0.97, () -> run(actions.launch3faster()))
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(83.000, 13.000),
                        new Pose(90.000, 18.000)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 30)))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        final double WAIT_AFTER_SHOOT = 0.8;

        // ----------------------------
        // WAIT-PHASE RPM HOLD (Option B via waitActions):
        // Add a long-running hold action at WAIT start for each shoot PathChainTask.
        // ----------------------------
        PathChainTask path1Task = new PathChainTask(path1, 1)
                .addWaitAction(0, actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 9999));
        tasks.add(path1Task);
        addPath(path2, 0.0);

        PathChainTask path3Task = new PathChainTask(path3, WAIT_AFTER_SHOOT)
                .addWaitAction(0, actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 9999));
        tasks.add(path3Task);
        addPath(path4, 0.0);

        PathChainTask path5Task = new PathChainTask(path5, WAIT_AFTER_SHOOT)
                .addWaitAction(0, actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 9999));
        tasks.add(path5Task);
        addPath(path6, 0.0);

        addPath(path6_5, 0.0);

        PathChainTask path7Task = new PathChainTask(path7, WAIT_AFTER_SHOOT)
                .addWaitAction(0, actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 9999));
        tasks.add(path7Task);
        addPath(path8, 0.0);

        PathChainTask path9Task = new PathChainTask(path9, WAIT_AFTER_SHOOT)
                .addWaitAction(0, actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 9999));
        tasks.add(path9Task);
        addPath(path10, 0.0);

        PathChainTask path12Task = new PathChainTask(path12far, 1)
                .addWaitAction(0, actions.holdShooterAtRPMclose(SHOOTER_HOLD_RPM, 9999));
        tasks.add(path12Task);
        addPath(leave, 0.0);
    }

    // =========================================================
    // ==================== TURRET (Predictive) ================
    // =========================================================

    private void updateTurret() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getAimPoseForTurret();

        double dx = targetX - aimPose.getX();
        double dy = targetY - aimPose.getY();

        double angleField = Math.atan2(dy, dx);
        double turretRad = normalize(angleField - aimPose.getHeading());

        double turretDeg = Math.toDegrees(turretRad) + TURRET_TRIM_DEG;
        turretDeg = Math.max(-TURRET_MAX_DEG, Math.min(TURRET_MAX_DEG, turretDeg));

        double servoPos;
        if (turretDeg >= 0) {
            servoPos = TURRET_CENTER +
                    (turretDeg / TURRET_MAX_DEG) * (TURRET_RIGHT - TURRET_CENTER);
        } else {
            servoPos = TURRET_CENTER -
                    (Math.abs(turretDeg) / TURRET_MAX_DEG) * (TURRET_CENTER - TURRET_LEFT);
        }

        servoPos = Math.max(0, Math.min(1, servoPos));

        double t1 = servoPos + TURRET_BACKLASH;
        t1 = Math.max(0, Math.min(1, t1));

        turret1.setPosition(t1);
        turret2.setPosition(servoPos);
    }

    private Pose getAimPoseForTurret() {
        Pose live = follower.getPose();

        // If we're in WAIT phase, aim live (robot is stopped / holding end)
        if (taskPhase != 0) return live;

        double t = follower.getCurrentTValue();
        if (t >= TURRET_LIVE_T) return live;

        PathChain active = getActivePathIfAny();
        if (active == null) return live;

        // Predict ONLY on shoot paths
        if (active == path1)     return poseAtEndOfPath1();
        if (active == path3)     return poseAtEndOfPath3();
        if (active == path5)     return poseAtEndOfPath5();
        if (active == path7)     return poseAtEndOfPath7();
        if (active == path9)     return poseAtEndOfPath9();
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

    // --- Predicted end poses (use endpoints you already use in paths) ---
    private Pose poseAtEndOfPath1() {
        // Line: (84,8)->(83.421,87.421), reversed
        double tangent = Math.atan2(87.421 - 8.000, 83.421 - 84.000);
        double h = normalize(tangent + Math.PI);
        return new Pose(83.421, 87.421, h);
    }

    private Pose poseAtEndOfPath3() {
        // Line: (123,80)->(83.208,87.421), reversed
        double tangent = Math.atan2(87.421 - 80.000, 83.208 - 123.000);
        double h = normalize(tangent + Math.PI);
        return new Pose(83.208, 87.421, h);
    }

    private Pose poseAtEndOfPath5() {
        // Approx last segment of curve ends at (83.421,87.634), reversed
        // Use tangent approximation based on last control->end segment:
        // control2: (85.335,35.734) to end (83.421,87.634)
        double tangent = Math.atan2(87.634 - 35.734, 83.421 - 85.335);
        double h = normalize(tangent + Math.PI);
        return new Pose(83.421, 87.634, h);
    }

    private Pose poseAtEndOfPath7() {
        // Line: (123,66)->(86.824,89.335), reversed
        double tangent = Math.atan2(89.335 - 66.000, 86.824 - 123.000);
        double h = normalize(tangent + Math.PI);
        return new Pose(86.824, 89.335, h);
    }

    private Pose poseAtEndOfPath9() {
        // Line: (131,20)->(83.421,87.634), reversed
        double tangent = Math.atan2(87.634 - 20.000, 83.421 - 131.000);
        double h = normalize(tangent + Math.PI);
        return new Pose(83.421, 87.634, h);
    }

    private Pose poseAtEndOfPath12far() {
        // Constant heading -360
        return new Pose(83.000, 13.000, Math.toRadians(-360));
    }

    private static double normalize(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
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
}
