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
@Autonomous(name = "1 - Redgatetest")
@Disabled
public class Redgatetest extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    // =========================
    // Turret target (RED) + turret constants
    // =========================
    public static double targetX = 122.0;
    public static double targetY = 125.0;

    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 140.0;  // deg left/right from center

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    // Predictive until t >= 0.9, then live
    private static final double TURRET_LIVE_T = 0.90;

    // =========================
    // Shooter settings
    // =========================
    private static final double TARGET_RPM = 1120.0;

    // =========================
    // Path decel settings
    // =========================
    private static final double GLOBAL_DECEL = 0.5;

    // =========================
    // Timing rules
    // =========================
    private static final double SHOOT_T = 0.95; // changed to 0.95 everywhere
    private static final double SHOOT_DELAY_S = 0.1;

    @Override
    public void init() {
        super.init();

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
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servo init (match your other autos)
        hood1.setPosition(0.475);
        turret1.setPosition(0.875);
        turret2.setPosition(0.875);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);

        // Indexer init baseline
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

        // Starting pose = Path1 start pose
        follower.setStartingPose(new Pose(87.421, 8.083, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        run(actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // Turret aiming
        updateTurret();

        // Shooter hold is no longer calculated/held in loop().

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
        // PATH 1 (SHOOT) - line, reversed
        // =========================
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(87.421, 8.083),
                        new Pose(86.996, 84.656)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration(GLOBAL_DECEL)
                //.addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                //.addParametricCallback(0.5, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                //.addParametricCallback(1.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(0.1),
                        actions.launch3faster()
                )))
                .build();

        // =========================
        // PATH 2 (INTAKE) - no decel
        // =========================
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(86.996, 84.656),
                        new Pose(99.545, 60.408),
                        new Pose(125.00, 60.195)
                ))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                //.addParametricCallback(0.0, () -> run(actions.startIntake()))
                //.addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                //.addParametricCallback(0.5, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                //.addParametricCallback(1.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))

                .build();

        // =========================
        // PATH 3 (SHOOT) - curve, reversed
        // =========================
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(125.00, 60.195),
                        new Pose(99.332, 60.408),
                        new Pose(87.208, 84.230)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration(GLOBAL_DECEL)
                //.addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                //.addParametricCallback(0.5, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                //.addParametricCallback(1.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // =========================
        // PATH 4 (INTAKE) - heading wrap fix: 310->40 becomes -50->-320
        // =========================
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.208, 84.230),
                        new Pose(90.00, 71.00),
                        new Pose(132.00, 60.0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(-340))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.5, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(1.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .build();

        // =========================
        // PATH 5 (SHOOT) - heading wrap fix: 40->310 becomes -320->-50, reversed
        // =========================
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(132.00, 60.0),
                        new Pose(98.694, 60.620),
                        new Pose(87.208, 84.230)
                ))
                //.setLinearHeadingInterpolation(Math.toRadians(-320), Math.toRadians(-50))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.5, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.7, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))

                .addParametricCallback(0.9, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))

                .addParametricCallback(1.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .build();

        // =========================
        // PATH 6 (INTAKE) - no decel
        // =========================
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(87.208, 82.230),
                        new Pose(124.0, 84.230)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.5, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(1.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .build();

        // =========================
        // PATH 7 (SHOOT) - line, reversed
        // =========================
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(124.0, 84.230),
                        new Pose(87.421, 84.230)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.5, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .addParametricCallback(1.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .build();

        // =========================
        // PATH 8 (INTAKE)
        // =========================
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.421, 84.230),
                        new Pose(99.120, 34.883),
                        new Pose(134.854, 34.245)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.5, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(1.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .build();

        // =========================
        // PATH 9 (SHOOT) - curve, reversed
        // =========================
        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(134.854, 34.245),
                        new Pose(98.907, 34.458),
                        new Pose(87.208, 84.443)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.5, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))

                .addParametricCallback(0.7, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.8, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.9, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))
                .addParametricCallback(0.95, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))

                .addParametricCallback(SHOOT_T, () -> run(new SequentialAction(
                        new SleepAction(SHOOT_DELAY_S),
                        actions.launch3faster()
                )))
                .addParametricCallback(1.0, () -> run(actions.holdShooterAtRPMclose(TARGET_RPM, 30)))


                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Wait rules:
        // - Each shooting path waits 0.8s after
        // - Path1 waits 1.0s after

        PathChainTask path1Task = new PathChainTask(path1, 1.2)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path1Task); // shooting path 1 special

        PathChainTask path2Task = new PathChainTask(path2, 0)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path2Task); // shooting path 1 special

        PathChainTask path3Task = new PathChainTask(path3, 1.2)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path3Task); // shooting

        PathChainTask path4TaskA = new PathChainTask(path4, 1.2)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path4TaskA); // shooting path 1 special

        PathChainTask path5TaskA = new PathChainTask(path5, 1.2)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path5TaskA); // shooting

        PathChainTask path4TaskB = new PathChainTask(path4, 1.2)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path4TaskB); // shooting path 1 special

        PathChainTask path5TaskB = new PathChainTask(path5, 1.2)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path5TaskB); // shooting

        PathChainTask path6Task = new PathChainTask(path6, 0)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path6Task); // shooting path 1 special

        PathChainTask path7Task = new PathChainTask(path7, 1.2)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path7Task); // shooting

        PathChainTask path8Task = new PathChainTask(path8, 0)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path8Task); // shooting path 1 special

        PathChainTask path9Task = new PathChainTask(path9, 1.2)
                .addWaitAction(0, actions.holdShooterAtRPMclose(TARGET_RPM, 9999));
        tasks.add(path9Task); // shooting
    }

    // =========================
    // REQUIRED OVERRIDES (exactly as requested)
    // =========================
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
    // Turret aiming: predicted end until t>=0.9, then live tracking
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

        // Only while driving
        if (taskPhase != 0) return live;

        double t = follower.getCurrentTValue();
        if (t >= TURRET_LIVE_T) return live;

        PathChain active = getActivePathIfAny();
        if (active == null) return live;

        // Predict end pose only on SHOOTING paths: 1,3,5,7,9
        if (active == path1) return poseAtEndOfPath1();
        if (active == path3) return poseAtEndOfPath3();
        if (active == path5) return poseAtEndOfPath5();
        if (active == path7) return poseAtEndOfPath7();
        if (active == path9) return poseAtEndOfPath9();

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

    // --- Predicted end poses ---
    private Pose poseAtEndOfPath1() {
        // Line start->end, reversed
        double h = headingFromLine(87.421, 8.083, 86.996, 84.656, true);
        return new Pose(86.996, 84.656, h);
    }

    private Pose poseAtEndOfPath3() {
        // Curve end tangent approx: last control -> end, reversed
        double h = headingFromLine(99.332, 60.408, 87.208, 84.230, true);
        return new Pose(87.208, 84.230, h);
    }

    private Pose poseAtEndOfPath5() {
        // Curve end tangent approx: last control -> end, reversed
        double h = headingFromLine(98.694, 60.620, 87.208, 84.443, true);
        return new Pose(87.208, 84.443, h);
    }

    private Pose poseAtEndOfPath7() {
        // Line start->end, reversed
        double h = headingFromLine(126.984, 84.230, 87.421, 84.230, true);
        return new Pose(87.421, 84.230, h);
    }

    private Pose poseAtEndOfPath9() {
        // Curve end tangent approx: last control -> end, reversed
        double h = headingFromLine(98.907, 34.458, 87.208, 84.443, true);
        return new Pose(87.208, 84.443, h);
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
