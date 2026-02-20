package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
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

@Autonomous(name = "1 - AdamAutoRed21")
public class AdamAutoRed21 extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, grab3wide, leave, shootout, grabout;

    // =========================
    // MIRROR HELPERS (BLUE -> RED)
    // Mirror across the FIELD CENTERLINE in X:
    // (x, y, heading) -> (144 - x, y, PI - heading)
    // =========================
    public static double FIELD_SIZE_X_IN = 144.0;

    private static double mirrorX(double x) { return x; }

    private static double mirrorHeadingRad(double h) {
        return normalizeRadians(h);
    }

    // =========================
    // Goal + turret constants (RED)
    // RED targetX = 128
    // =========================
    public static double targetX = mirrorX(124.0);
    public static double targetY = 125.0;

    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 137;    // deg

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    // Keep identical behavior to RED code (mirrored)
    private static final double TURRET_LIVE_T = 0.85;

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

        // Starting pose mirrored from RED:
        // RED  start: (86.783, 8.934, 0)
        // BLUE start: (144 - 86.783 = 57.217, 8.934, PI)
        Pose startPose = new Pose(mirrorX(86.783), 8.934, mirrorHeadingRad(0.0));
        follower.setStartingPose(startPose);

        // Init should aim at the goal as well (hardset once here)
        setTurretToAimAtPose(startPose);

        buildPathChains();
        buildTaskList();

        shooterHoldUpdater = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double vR   = shootr.getVelocity();
                double vL   = shootl.getVelocity();
                double avgRpm = 0.5 * ((vR / 28.0) * 60.0 + (vL / 28.0) * 60.0);

                if (startBoostActive && avgRpm >= TARGET_RPM) {
                    startBoostActive = false;
                }

                double baseTarget = preloadBoostActive ? (TARGET_RPM + 70.0) : (TARGET_RPM + 20.0);
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

        // This is the BLUE mirror of your RED paths.
        // Mirror rule applied to every pose: x -> 144 - x, heading -> PI - heading.

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(86.783), 8.934),
                        new Pose(mirrorX(93.500), 11.911)
                ))
                .setConstantHeadingInterpolation(mirrorHeadingRad(Math.toRadians(0)))
                .setBrakingStrength(2)
                .addParametricCallback(0.9, () -> run(new SequentialAction(
                        new SleepAction(2.2),
                        actions.launch3faster()
                )))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(98.907), 11.911),
                        new Pose(mirrorX(126.0), 9.0)
                ))
                .setConstantHeadingInterpolation(mirrorHeadingRad(0.0))
                .setBrakingStrength(2.5)

                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        grabout = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(98.907), 11.911),
                        new Pose(mirrorX(126.0), 24.0)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(2.5)

                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        grab3wide = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose((mirrorX(98.907)), 11.911),
                        new Pose((mirrorX(128.0)), 9.00)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .setBrakingStrength(2)
                .addPath(new BezierLine(
                        new Pose((mirrorX(128.0)), 9.00),
                        new Pose((mirrorX(120.0)), 9.00)
                ))
                .setTangentHeadingInterpolation().setReversed()
                .setBrakingStrength(2)
                .addPath(new BezierLine(
                        new Pose((mirrorX(120.0)), 9.00),
                        new Pose((mirrorX(128.0)), 8.00)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(2)
                // Blue mirror of RED targetX=132 -> BLUE targetX=12
                .addParametricCallback(1.0, () -> targetX = mirrorX(128))
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(98.907), 11.911),
                        new Pose(mirrorX(105.0), 12.00)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(3)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(126.00), 9.00),
                        new Pose(mirrorX(128.898), 8.00)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(3)
                .build();

        // Path4 declared but unused in your RED snippet; leaving as-is (null)

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(126.0), 9.00),
                        new Pose(mirrorX(99.120), 11.699)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(3)
                .setReversed()
                .addParametricCallback(0.1, () -> run(actions.stopIntake()))

                .addParametricCallback(0.95, () -> run(new SequentialAction(
                        new SleepAction(0.1),
                        actions.launch3faster()
                )))
                .build();

        shootout = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(126.0), 20),
                        new Pose(mirrorX(99.120), 11.699)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(3)
                .setReversed()
                .addParametricCallback(0.95, () -> run(new SequentialAction(
                        new SleepAction(0.1),
                        actions.launch3faster()
                )))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        tasks.add(new PathChainTask(path1, 3.2));
        addPath(grab3wide, 0);
        //addPath(path3, 0);
        tasks.add(new PathChainTask(path5, 1.5));
        addCycle2to5();
        addPath(path2, 0.5);
        tasks.add(new PathChainTask(path5, 2.3));        addCycle2to5();
        addPath(grabout, 0.5);
        tasks.add(new PathChainTask(shootout, 1.5));
        addCycle2to5();

        tasks.add(new PathChainTask(leave, 0));
    }

    private void addCycle2to5() {
        addPath(path2, 0.5);
        tasks.add(new PathChainTask(path5, 1.5));
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

        if (active == path1) return poseAtEndOfPath1();
        if (active == path5) return poseAtEndOfPath5();
        if (active == shootout) return poseAtEndOfShootout();

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
        // path1 actually ends at mirrorX(93.500), 11.911 with constant mirrored heading
        return new Pose(mirrorX(93.500), 11.911, mirrorHeadingRad(Math.toRadians(0)));
    }

    private Pose poseAtEndOfPath5() {
        double h = headingFromLine(126.0, 10.422, 99.120, 11.699, true);
        double blueH = mirrorHeadingRad(h);
        return new Pose(mirrorX(99.120), 11.699, blueH);
    }

    private Pose poseAtEndOfShootout() {
        double h = headingFromLine(126.0, 24.0, 99.120, 11.699, true);
        double blueH = mirrorHeadingRad(h);
        return new Pose(mirrorX(99.120), 11.699, blueH);
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
        try { PoseStore.saveRed(follower.getPose()); } catch (Exception ignored) {}
        super.stop();
    }
}

