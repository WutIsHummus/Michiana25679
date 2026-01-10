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

@Autonomous(name = "1 - bluefar21nospike")
public class bluefar21nospike extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, sus, sus2, path3, path4, path5;

    // =========================
    // Mirror helpers (RED -> BLUE)
    // =========================
    private static double mx(double x) { return 144.0 - x; }

    // Heading mirror: blueHeading = PI - redHeading
    private static double mh(double headingRad) { return normalizeRadians(Math.PI - headingRad); }

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // =========================
    // Turret + goal constants (BLUE mirrored from your RED)
    // =========================
    // RED targetX=132 => BLUE targetX=144-132=12
    public static double targetX = mx(132.0); // 12.0
    public static double targetY = 125.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 137;    // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.027;

    // Keep your tuned calibration offset
    public static double TURRET_CAL_OFFSET_DEG = -1.5;

    // Shooter RPM (same as your RED opmode)
    private static final double SHOOT_RPM_HOLD = 1390;

    // One-time start boost behavior:
    // Start at 1700 RPM until measured shooter avg RPM exceeds 1400,
    // then drop target to 1400 for the rest of the auto.
    private static final double START_BOOST_RPM = 1700.0;
    private boolean startBoostDone = false;

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

        // Hood fixed at 0.45 (same)
        hood1.setPosition(0.45);

        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);
        indexfront.setPosition(RobotActions.INDEX_FRONT_EXTENDED);
        indexback.setPosition(RobotActions.INDEX_BACK_RETRACTED);

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                indexfront, indexback,
                hardwareMap.voltageSensor.iterator().next()
        );
        run(actions.safeindexer());

        // Mirror your RED starting pose (86.570, 8.934, 0) -> BLUE: (57.430, 8.934, 180)
        follower.setStartingPose(new Pose(mx(86.570), 8.934, mh(Math.toRadians(0))));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
        startBoostDone = false;
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // Turret auto-aim every loop (BLUE-side constraint)
        updateTurretAutoAimBlueSide();

        // Compute shooter RPM avg for boost gating + telemetry
        double vR   = shootr.getVelocity();
        double vL   = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avg  = 0.5 * (rpmR + rpmL);

        // One-time boost logic at beginning of auto
        if (!startBoostDone && avg > SHOOT_RPM_HOLD) {
            startBoostDone = true;
        }
        double targetRpmThisLoop = startBoostDone ? SHOOT_RPM_HOLD : START_BOOST_RPM;

        run(actions.holdShooterAtRPMclose(targetRpmThisLoop, 30));

        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("BoostDone", startBoostDone);
        telemetry.addData("TargetRPM", "%.0f", targetRpmThisLoop);
        telemetry.addData("RPM R", "%.0f", rpmR);
        telemetry.addData("RPM L", "%.0f", rpmL);
        telemetry.addData("RPM avg", "%.0f", avg);

        telemetry.addLine("=== TURRET AUTO-AIM (BLUE) ===");
        Pose pose = follower.getPose();
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Target", "(%.1f, %.1f)", targetX, targetY);

        double turretDeg = computeTurretAngleDeg(pose);
        telemetry.addData("Turret angle (deg)", "%.2f", turretDeg);
        telemetry.addData("Trim (deg)", "%.2f", turretTrimDeg);
        telemetry.addData("CalOffset (deg)", "%.2f", TURRET_CAL_OFFSET_DEG);

        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // This is a direct mirror of your RED buildPathChains():
        // X' = 144 - X; Y unchanged; Heading' = PI - Heading

        // === Path1: (86.570, 8.934) -> (86.783, 15.315), heading 0 ===
        // BLUE: (57.430, 8.934) -> (57.217, 15.315), heading 180
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mx(86.570), 8.934),
                        new Pose(mx(86.783), 15.315)))
                .setConstantHeadingInterpolation(mh(Math.toRadians(0)))
                .setGlobalDeceleration(0.5)
                .build();

        // === Path2: (86.783,15.315)->(104,10)->(124,8), heading 0->0 ===
        // BLUE: (57.217,15.315)->(40,10)->(20,8), heading 180->180
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(mx(86.783), 15.315),
                        new Pose(mx(104), 10),
                        new Pose(mx(124), 8)))
                .setLinearHeadingInterpolation(mh(Math.toRadians(0)), mh(Math.toRadians(0)))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // sus: (124,8)->(129,9) h 10->350
        // BLUE: (20,8)->(15,9) h 170->190
        sus = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mx(124), 8),
                        new Pose(mx(129), 9)))
                .setLinearHeadingInterpolation(mh(Math.toRadians(10)), mh(Math.toRadians(350)))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // sus2: (129,9)->(132,8) h 350->0
        // BLUE: (15,9)->(12,8) h 190->180
        sus2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mx(129), 9),
                        new Pose(mx(132), 8)))
                .setLinearHeadingInterpolation(mh(Math.toRadians(350)), mh(Math.toRadians(0)))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path3: (132,8)->(86.783,13), heading 0, shoot callback ===
        // BLUE: (12,8)->(57.217,13), heading 180, shoot callback
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mx(132), 8),
                        new Pose(mx(86.783), 13)))
                .setConstantHeadingInterpolation(mh(Math.toRadians(0)))
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.95, () -> run(new SequentialAction(
                        new SleepAction(0.1),
                        actions.launch3faster()
                )))
                .build();

        // === Path4: (86.783,15.315)->(122,35.734), tangent heading, intake ===
        // BLUE: (57.217,15.315)->(22,35.734)
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mx(86.783), 15.315),
                        new Pose(mx(122), 35.734)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path5: (122,35.734)->(86.783,15.315), heading 0, far shot callback ===
        // BLUE: (22,35.734)->(57.217,15.315), heading 180, far shot callback
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mx(122), 35.734),
                        new Pose(mx(86.783), 15.315)
                ))
                .setConstantHeadingInterpolation(mh(Math.toRadians(0)))
                .setGlobalDeceleration(0.53)
                .addParametricCallback(0.95, () -> run(new SequentialAction(
                        new SleepAction(0.4),
                        actions.launch3far()
                )))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // === Path1: drive to shooter, WAIT, then LAUNCH ===
        PathChainTask path1Task = new PathChainTask(path1, 2.5)
                .addWaitAction(
                        0,
                        new SequentialAction(
                                new SleepAction(1.3),
                                actions.launch3faster()
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path1Task);

        // === Cycles (same sequence) ===
        addPath(path2, 0.3);
        //addPath(sus, 0);
        //addPath(sus2, 0);
        addPath(path3, 1);

        addPath(path2, 0);
        addPath(path3, 1);

        addPath(path2, 0);
        addPath(path3, 1);

        addPath(path2, 0);
        addPath(path3, 1);

        addPath(path2, 0);
        addPath(path3, 1);

        addPath(path2, 0);
        addPath(path3, 1);

        addPath(path2, 0);
    }

    @Override
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected boolean isTurning() {
        return false;
    }

    @Override
    protected double getCurrentTValue() {
        return follower.getCurrentTValue();
    }

    @Override
    protected void startPath(PathChainTask task) {
        follower.followPath((PathChain) task.pathChain, true); // holdEnd = true
    }

    @Override
    protected void startTurn(TurnTask task) {
        // Not used
    }

    // =========================
    // Turret calculations (BLUE constraint applied)
    // =========================

    /**
     * Returns the turret angle in degrees (robot-relative) with trim + calibration applied.
     */
    private double computeTurretAngleDeg(Pose pose) {
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();

        double angleToTargetField = Math.atan2(dy, dx);
        double turretAngleRad = normalizeRadians(angleToTargetField - pose.getHeading());

        return Math.toDegrees(turretAngleRad) + turretTrimDeg + TURRET_CAL_OFFSET_DEG;
    }

    /**
     * Auto-aim turret from pose to BLUE target, with BLUE-side servo constraint:
     * only allow lower-than-center side (<= turretCenterPosition).
     */
    private void updateTurretAutoAimBlueSide() {
        if (turret1 == null || turret2 == null || follower == null) return;

        Pose pose = follower.getPose();

        double turretAngleDeg = computeTurretAngleDeg(pose);

        // Clamp to mechanical turret angle
        double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDeg));

        // Map angle -> servo
        double servoPosition;
        if (clampedAngle >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        // Clamp servo to [0,1]
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // BLUE SIDE constraint: only lower-than-center
        servoPosition = Math.min(turretCenterPosition, servoPosition);

        // Backlash comp on turret1
        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        // Apply BLUE constraint again after backlash
        turret1Pos = Math.min(turretCenterPosition, turret1Pos);

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);
    }

    @Override
    public void stop() {
        try {
            PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}
