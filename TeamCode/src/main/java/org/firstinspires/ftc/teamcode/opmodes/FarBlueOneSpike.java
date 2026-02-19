package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

@Autonomous(name = "FarBlueOneSpike")
public class FarBlueOneSpike extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    // ===== Paths (same order you posted) =====
    private PathChain path1, path2, path3, path4, path5, path6, path7;

    // ===== Goal + turret constants (BLUE) =====
    // Keep identical to your current Blue auto behavior
    public static double targetX = 13.0;   // Blue goal X
    public static double targetY = 125.0;

    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 137;    // deg

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    private static final double TURRET_LIVE_T = 0.9;

    private static final double TARGET_RPM = 1370;
    private static final double HOOD_POS   = 0.44;
    private static final double START_BOOST_RPM = 8000.0;

    private double desiredHoldRpm = TARGET_RPM;
    private Action currentHoldAction = null;
    private double currentHoldRpm = Double.NaN;
    private Action shooterHoldUpdater = null;
    private boolean startBoostActive = true;
    private double pendingHoldRpm = TARGET_RPM;
    private boolean holdUpdatePending = false;
    private boolean preloadBoostActive = true;

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

        // Starting pose from your visualizer
        Pose startPose = new Pose(57.702, 9.489, Math.toRadians(180));
        follower.setStartingPose(startPose);

        // Hard-aim once at init
        setTurretToAimAtPose(startPose);

        buildPathChains();
        buildTaskList();

        shooterHoldUpdater = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double vR = shootr.getVelocity();
                double vL = shootl.getVelocity();
                double avgRpm = 0.5 * ((vR / 28.0) * 60.0 + (vL / 28.0) * 60.0);

                if (startBoostActive && avgRpm >= TARGET_RPM) {
                    startBoostActive = false;
                }

                double baseTarget = preloadBoostActive ? (TARGET_RPM + 30.0) : (TARGET_RPM + 20.0);
                if (!follower.isBusy() && currentTaskIndex > 0) {
                    preloadBoostActive = false;
                }

                desiredHoldRpm = startBoostActive ? START_BOOST_RPM : baseTarget;
                pendingHoldRpm = desiredHoldRpm;
                holdUpdatePending = true;
                return true;
            }
        };
        addBackgroundAction(shooterHoldUpdater);
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
        desiredHoldRpm = TARGET_RPM;
        startBoostActive = true;
        preloadBoostActive = true;
        updateShooterHold(desiredHoldRpm);
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        updateTurretAim();
        applyPendingHoldUpdate();

        runTasks();
    }

    @Override
    protected void buildPathChains() {

        // ===== Path 1 =====
        // BezierLine: (57.702, 9.489) -> (51.108, 15.793)
        // Linear heading: 180 -> 180
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(57.702, 9.489),
                        new Pose(52.0, 14.0)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setBrakingStrength(2.0)
                // Path1 ends in shoot (preload)
                .addParametricCallback(0.85, () -> run(new SequentialAction(
                        new SleepAction(2.3),
                        actions.launch3faster()
                )))
                .build();

        // ===== Path 2 =====
        // BezierCurve: (51.108, 15.793) -> CP (55.478, 37.817) -> (10.158, 34.697)
        // Constant heading: 180
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(52.0, 14.0),
                        new Pose(55.478, 37.817),
                        new Pose(13.0, 34.697)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(2.5)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // ===== Path 3 =====
        // BezierLine: (10.158, 34.697) -> (50.871, 15.675)
        // Tangent heading, reversed = true
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(13.0, 34.697),
                        new Pose(52.0, 18.0)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(3.0)
                .addParametricCallback(0.55, () -> run(actions.stopIntake()))
                // shoot phase
                .addParametricCallback(0.95, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        // ===== Path 4 =====
        // BezierLine: (50.871, 15.675) -> (12.755, 10.970)
        // Tangent heading
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(52.0, 18.0),
                        new Pose(12.755, 10.970)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(2.5)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // ===== Path 5 =====
        // BezierLine: (12.755, 10.970) -> (49.430, 12.028)
        // Tangent heading, reversed = true
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(12.755, 10.970),
                        new Pose(49.430, 12.028)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStrength(3.0)
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                // shoot phase
                .addParametricCallback(0.92, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();

        // ===== Path 6 =====
        // BezierLine: (49.430, 12.028) -> (12.174, 10.900)
        // Tangent heading
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(49.430, 12.028),
                        new Pose(18.0, 10.900)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(2.5)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // ===== Path 7 =====
        // BezierLine: (12.174, 10.900) -> (49.378, 11.576)
        // Tangent heading, reversed = true
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(18.0, 10.900),
                        new Pose(49.378, 11.576)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStrength(3.0)
                .addParametricCallback(0.55, () -> run(actions.stopIntake()))
                // final shoot (or last dump) based on your existing pattern
                .addParametricCallback(0.92, () -> run(new SequentialAction(
                        new SleepAction(0.10),
                        actions.launch3faster()
                )))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Same path order, alternating intake/shoot phases after Path1.
        tasks.add(new PathChainTask(path1, 3.3));  // shoot preload

        tasks.add(new PathChainTask(path2, 0.5));  // intake
        tasks.add(new PathChainTask(path3, 1.5));  // shoot

        tasks.add(new PathChainTask(path4, 0.5));  // intake
        tasks.add(new PathChainTask(path5, 1.5));  // shoot

        tasks.add(new PathChainTask(path6, 0));  // intake
        tasks.add(new PathChainTask(path7, 1.5));  // shoot / finish
        tasks.add(new PathChainTask(path6, 0));  // intake
        tasks.add(new PathChainTask(path7, 1.5));  // shoot / finish
        tasks.add(new PathChainTask(path6, 0));  // intake
        tasks.add(new PathChainTask(path7, 1.5));  // shoot / finish
        tasks.add(new PathChainTask(path6, 0));  // intake
        tasks.add(new PathChainTask(path7, 1.5));  // shoot / finish
        tasks.add(new PathChainTask(path6, 0));  // intake


    }

    // ===== Shooter hold (same behavior) =====

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

    private void applyPendingHoldUpdate() {
        if (!holdUpdatePending) return;
        holdUpdatePending = false;
        updateShooterHold(pendingHoldRpm);
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

    // ===== Turret aiming (same predictive/live logic) =====

    private void updateTurretAim() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getAimPoseForTurret();
        setTurretToAimAtPose(aimPose);
    }

    private Pose getAimPoseForTurret() {
        Pose live = follower.getPose();

        // keep identical gating logic to your current auto
        if (taskPhase != 0) return live;

        double t = follower.getCurrentTValue();
        if (t >= TURRET_LIVE_T) return live;

        PathChain active = getActivePathIfAny();
        if (active == null) return live;

        // Predictive aim: shoot paths aim at their END pose until live threshold
        if (active == path1) return poseAtEndOfPath1();
        if (active == path3) return poseAtEndOfPath3();
        if (active == path5) return poseAtEndOfPath5();
        if (active == path7) return poseAtEndOfPath7();

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

    private Pose poseAtEndOfPath1() {
        return new Pose(51.108, 15.793, Math.toRadians(180));
    }

    private Pose poseAtEndOfPath3() {
        // reversed line: (10.158,34.697)->(50.871,15.675)
        // end heading is tangent + PI because reversed
        double h = headingFromLine(10.158, 34.697, 50.871, 15.675, true);
        return new Pose(50.871, 15.675, h);
    }

    private Pose poseAtEndOfPath5() {
        double h = headingFromLine(12.755, 10.970, 49.430, 12.028, true);
        return new Pose(49.430, 12.028, h);
    }

    private Pose poseAtEndOfPath7() {
        double h = headingFromLine(12.174, 10.900, 49.378, 11.576, true);
        return new Pose(49.378, 11.576, h);
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
        try { PoseStore.saveBlue(follower.getPose()); } catch (Exception ignored) {}
        super.stop();
    }
}

