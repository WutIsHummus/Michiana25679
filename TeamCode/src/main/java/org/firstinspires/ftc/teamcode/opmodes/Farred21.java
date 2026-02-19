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

@Autonomous(name = "1 - redfar21nospike")
@Disabled
public class Farred21 extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, sus, sus2;

    // =========================
    // Turret + goal constants (RED)
    // =========================

    // RED target (original was 128,125). Apply your misaligned-goal compensation:
    // subtract 3 inches from RED target X.
    public static double targetX = 132.0; // 125.0
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

    // Shooter RPM (your new request)
    private static final double SHOOT_RPM_HOLD = 1390.0;

    // One-time start boost behavior:
    // Start at 1700 RPM until measured shooter avg RPM exceeds 1350,
    // then drop target to 1350 for the rest of the auto.
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

        // Your request: hood fixed at 0.45 for this auto
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

        // Mirror BLUE starting pose (57.430, 8.934, 180) across X=72:
        // X' = 144 - 57.430 = 86.570
        // Heading' = 180 - 180 = 0
        follower.setStartingPose(new Pose(86.570, 8.934, Math.toRadians(0)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
        startBoostDone = false; // ensure boost runs only once per opmode start
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // Turret auto-aim every loop (RED-side mirrored logic)
        updateTurretAutoAimRedSide();

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

        // Keep shooter spun up (your actions PID)
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

        telemetry.addLine("=== TURRET AUTO-AIM (RED) ===");
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

        // Mirroring rule applied to ALL BLUE path points:
        // X' = 144 - X ; Y unchanged
        // Heading mirroring used below:
        // h' = 180 - h (mod 360)
        //
        // BLUE reference points:
        //  start: (57.430, 8.934, 180) -> RED: (86.570, 8.934, 0)

        // === Path1: (57.430, 8.934) -> (57.217, 15.315) ===
        // RED: (86.570, 8.934) -> (86.783, 15.315)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(86.570, 8.934),
                        new Pose(86.783, 15.315)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration(0.5)
                .build();

        // === Path2: shooter -> stack ===
        // BLUE: (57.217,15.315) -> (40,10) -> (20,8)
        // RED:  (86.783,15.315) -> (104,10) -> (124,8)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(86.783, 15.315),
                        new Pose(104, 10),
                        new Pose(124, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Optional “sus” lines mirrored too
        // BLUE: (20,8)->(15,9) h 170->190
        // RED:  (124,8)->(129,9) h 10->350
        sus = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(124, 8),
                        new Pose(129, 9)))
                .setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(350))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // BLUE: (15,9)->(12,8) h 190->180
        // RED:  (129,9)->(132,8) h 350->0
        sus2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(129, 9),
                        new Pose(132, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(0))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path3: stack -> shooter (shoot callback at 0.95) ===
        // BLUE: (12,8)->(57.217,13)
        // RED:  (132,8)->(86.783,13)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(124, 8),
                        new Pose(86.783, 13)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.95, () -> run(new SequentialAction(
                        new SleepAction(0.1),
                        actions.launch3faster()
                )))
                .build();

        // === Path4: shooter -> stack 2 ===
        // BLUE: (57.217,15.315)->(22,35.734)
        // RED:  (86.783,15.315)->(122,35.734)
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(86.783, 15.315),
                        new Pose(122, 35.734)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path5: stack 2 -> shooter ===
        // BLUE: (22,35.734)->(57.217,15.315)
        // RED:  (122,35.734)->(86.783,15.315)
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(122, 35.734),
                        new Pose(86.783, 15.315)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
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

        // === Cycles ===
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
    // Turret calculations (RED constraint applied)
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
     * Auto-aim turret from pose to RED target, with RED-side servo constraint:
     * only allow higher-than-center side (>= turretCenterPosition).
     */
    private void updateTurretAutoAimRedSide() {
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

        // RED SIDE constraint: only higher-than-center
        servoPosition = Math.max(turretCenterPosition, servoPosition);

        // Backlash comp on turret1
        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        // Apply RED constraint again after backlash
        turret1Pos = Math.max(turretCenterPosition, turret1Pos);

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);
    }

    @Override
    public void stop() {
        try {
            PoseStore.saveRed(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}

