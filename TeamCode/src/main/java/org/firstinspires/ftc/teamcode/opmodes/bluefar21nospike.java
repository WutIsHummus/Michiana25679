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

    private PathChain path1, path2, path3, path4, path5, sus, sus2;

    // =========================
    // Turret + goal constants (BLUE)
    // =========================

    // Field target (BLUE) — mirror of RED (128,125) across field width 144 => (16,125)
    public static double targetX = 16.0;
    public static double targetY = 125.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 137;    // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.027;

    // Small calibration offset (deg). Keep as your tuned value.
    public static double TURRET_CAL_OFFSET_DEG = -1.5;

    // Shooter RPM (kept as your original)
    private static final double SHOOT_RPM_HOLD = 1350;

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

        follower.setStartingPose(new Pose(57.430, 8.934, Math.toRadians(180)));

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

        // Turret auto-aim every loop
        updateTurretAutoAimBlueSide();

        // Keep shooter spun up
        run(actions.holdShooterAtRPMclose(SHOOT_RPM_HOLD, 30));

        runTasks();

        double vR   = shootr.getVelocity();
        double vL   = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avg  = 0.5 * (rpmR + rpmL);

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("RPM R", "%.0f", rpmR);
        telemetry.addData("RPM L", "%.0f", rpmL);
        telemetry.addData("RPM avg", "%.0f", avg);

        telemetry.addLine("=== TURRET AUTO-AIM ===");
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

        // === Path1: (57.430, 8.934) -> (57.217, 15.315) ===
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(57.430, 8.934),
                        new Pose(57.217, 15.315)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setGlobalDeceleration(0.5)
                .build();

        // === Path2: shooter -> stack ===
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(57.217, 15.315),
                        new Pose(40, 10),
                        new Pose(20, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        sus = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20, 8),
                        new Pose(15, 9)))
                .setLinearHeadingInterpolation(Math.toRadians(170), Math.toRadians(190))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        sus2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15, 9),
                        new Pose(12, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(180))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path3: stack -> shooter (shoot callback at 0.9) ===
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(12, 8),
                        new Pose(57.217, 13)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setGlobalDeceleration(0.54)
                // REPLACED: launch3far -> launch3faster
                .addParametricCallback(0.95, () -> run(new SequentialAction(new SleepAction(0.4),
                        actions.launch3far())))

                .build();

        // === Path4: shooter -> stack 2 (curve) ===
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(57.217, 15.315),
                        new Pose(22, 35.734)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path5: stack 2 -> shooter (line) ===
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22, 35.734),
                        new Pose(57.217, 15.315)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setGlobalDeceleration(0.53)
                // REPLACED: launch3far -> launch3faster
                .addParametricCallback(0.95, () -> run(new SequentialAction(new SleepAction(0.4),
                        actions.launch3far())))

                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // === Path1: drive to shooter, WAIT, then LAUNCH (already launch3faster) ===
        PathChainTask path1Task = new PathChainTask(path1, 3.2)
                .addWaitAction(
                        0,
                        new SequentialAction(
                                new SleepAction(1.5),
                                actions.launch3faster()
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path1Task);


        // === Cycles ===
        addPath(path2, 0.3);
        //addPath(sus, 0);
        //addPath(sus2, 0);
        addPath(path3, 1.4);

        addPath(path2, 0);
        addPath(path3, 1.4);

        addPath(path2, 0);
        addPath(path3, 1.4);

        addPath(path2, 0);
        addPath(path3, 1.4);

        addPath(path2, 0);
        addPath(path3, 1.4);

        addPath(path2, 0);
        addPath(path3, 1.4);

        addPath(path2, 0); // final intake move as in your original
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

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

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
