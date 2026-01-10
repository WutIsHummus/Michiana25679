package org.firstinspires.ftc.teamcode.opmodes;

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

@Autonomous(name = "1 - Bluefar18consistent")
public class Bluefar18consistent extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5;

    // =========================
    // Goal + turret constants (BLUE)
    // =========================
    public static double targetX = 144.0 - 130.0; // 19
    public static double targetY = 125.0;

    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 137;  // deg

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    // Requirement: predictive aim until t >= 0.95, then active tracking
    private static final double TURRET_LIVE_T = 0.5;

    // Shooter + hood requirement
    private static final double TARGET_RPM = 1380;
    private static final double HOOD_POS   = 0.44;

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

        // Indexer baseline (consistent with your other autos)
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

        // Starting pose = first pose of Path1
        Pose startPose = new Pose(57.217, 8.934, Math.toRadians(180));
        follower.setStartingPose(startPose);

        // Init should aim at the goal as well (hardset once here)
        setTurretToAimAtPose(startPose);

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

        // Turret aim: predictive until t>=0.95, then live tracking
        updateTurretAim();

        // Hold shooter RPM
        run(actions.holdShooterAtRPMfar(TARGET_RPM, 30));

        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // Path1
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(57.217, 8.934),
                        new Pose(55, 11.911)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setGlobalDeceleration(0.5)
                .addParametricCallback(0.9, () -> run(new SequentialAction(
                        new SleepAction(2.2),
                        actions.launch3faster()
                )))
                .build();

        // Path2
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(45.093, 11.911),
                        new Pose(18, 10.422)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(1.5)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(14, 10.422),
                        new Pose(15.102, 10.422)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(2)
                .setReversed()
                .build();



        // Path5 (shoot at t=0.95)
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(18, 10.422),
                        new Pose(44.880, 11.699)
                ))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(2.5)
                .setReversed()
                .addParametricCallback(0.95, () -> run(new SequentialAction(
                new SleepAction(0.18),
                actions.launch3faster()
                )))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Run path1, then wait 3 seconds
        tasks.add(new PathChainTask(path1, 3.2));
        addPath(path2, 0.2);
        //addPath(path3, 0.0);
        tasks.add(new PathChainTask(path5, 1.0));

        // Repeat paths 2-5 four times, no loops
        // Requirement: after end of path5 wait 1s
        addCycle2to5();
        addCycle2to5();
        addCycle2to5();
        addCycle2to5();
        addCycle2to5();
        tasks.add(new PathChainTask(path2,0));

    }

    private void addCycle2to5() {
        addPath(path2, 0.1);
        //addPath(path3, 0.0);
        //addPath(path4, 0.0);
        tasks.add(new PathChainTask(path5, 1.6));
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
    // Turret aiming (predictive -> live at t>=0.95)
    // =========================

    private void updateTurretAim() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getAimPoseForTurret();
        setTurretToAimAtPose(aimPose);
    }

    /**
     * Predictive only for the “shooting relevant” paths in this auto:
     * - Path1: you shoot near the end (t=0.9 + delayed sequence)
     * - Path5: you shoot at t=0.95
     * Otherwise, just aim live.
     */
    private Pose getAimPoseForTurret() {
        Pose live = follower.getPose();

        // only while driving
        if (taskPhase != 0) return live;

        double t = follower.getCurrentTValue();
        if (t >= TURRET_LIVE_T) return live;

        PathChain active = getActivePathIfAny();
        if (active == null) return live;

        if (active == path1) return poseAtEndOfPath1();
        if (active == path5) return poseAtEndOfPath5();

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
        // End pose is known (linear heading 180)
        return new Pose(45.093, 11.911, Math.toRadians(180));
    }

    private Pose poseAtEndOfPath5() {
        // Line from (10.210,10.422) -> (44.880,11.699), reversed => heading = tangent + pi
        double h = headingFromLine(10.210, 10.422, 44.880, 11.699, true);
        return new Pose(44.880, 11.699, h);
    }

    /**
     * Compute turret angle from a pose to the goal, clamp, map to servo, apply backlash on turret1.
     */
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
        try { PoseStore.save(follower.getPose()); } catch (Exception ignored) {}
        super.stop();
    }
}
