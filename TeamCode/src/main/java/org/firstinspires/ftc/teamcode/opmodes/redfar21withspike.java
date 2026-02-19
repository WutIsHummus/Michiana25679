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

@Autonomous(name = "1 - redfar21withspike")
@Disabled
public class redfar21withspike extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, sus, sus2;

    // --- Turret constants ---

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 137;    // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.027;

    // Mirrored OG tuned angle:
    // If Blue fixed angle was -68.55°, mirroring flips sign.
    public static double FIXED_TURRET_ANGLE_DEG = +68.55;

    // === Field target (RED) ===
    // Blue auto used: targetX = 144 - 128 = 16.0, targetY = 125.0
    // Mirroring back to Red: targetX = 128.0, targetY unchanged.
    public static double targetX = 131;
    public static double targetY = 125.0;

    // Mirrored calibration offset (sign flipped to match mirrored turret angle direction)
    public static double TURRET_CAL_OFFSET_DEG = +1.5;

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

        // Shooter motor direction
        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servo init
        hood1.setPosition(0.47);
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

        // === Mirrored start pose ===
        // Blue: (57.430, 8.934, 180°)
        // Mirror X: 144 - 57.430 = 86.570
        // Mirror heading: 180° -> 0°
        follower.setStartingPose(new Pose(86.570, 8.934, Math.toRadians(0)));

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

        // Continuously auto-aim turret to target using localization
        updateTurretAutoAim();

        // keep shooter spun up (tune as needed)
        run(actions.holdShooterAtRPMclose(1325, 30));

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

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double fieldAngle = Math.atan2(dy, dx);
        double turretAngleRad = fieldAngle - pose.getHeading();
        while (turretAngleRad > Math.PI)  turretAngleRad -= 2 * Math.PI;
        while (turretAngleRad < -Math.PI) turretAngleRad += 2 * Math.PI;

        double turretAngleDeg = Math.toDegrees(turretAngleRad)
                + turretTrimDeg + TURRET_CAL_OFFSET_DEG;

        telemetry.addData("Turret angle (deg)", "%.2f", turretAngleDeg);
        telemetry.addData("Trim (deg)", "%.2f", turretTrimDeg);
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {
        // Mirroring rule used here:
        //   X' = 144 - X
        //   Y' = Y
        //   headingDeg' = (180 - headingDeg + 360) % 360

        // Shooter pose (blue): (57.217, 15.315)
        // Shooter pose (red mirrored): (86.783, 15.315)
        // since 144 - 57.217 = 86.783

        // === Path1: (57.430, 8.934) -> (57.217, 15.315) ===
        // Mirrored: (86.570, 8.934) -> (86.783, 15.315), heading 180 -> 0
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(86.570, 8.934),
                        new Pose(86.783, 15.315)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration(0.5)
                .build();

        // === Path2: (57.217, 15.315) -> (12, 8) ===
        // Mirrored: (86.783, 15.315) -> (132, 8), heading 180 -> 0
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(86.783, 15.315),
                        new Pose(132, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // sus: (12,8)->(15,9) with 170->190
        // Mirrored: (132,8)->(129,9) with heading 170->10, 190->350
        sus = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(132, 8),
                        new Pose(129, 9)))
                .setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(350))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // sus2: (15,9)->(12,8) with 190->180
        // Mirrored: (129,9)->(132,8) with heading 350->0
        sus2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(129, 9),
                        new Pose(132, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(0))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path3: (12, 8) -> (57.217, 13) ===
        // Mirrored: (132,8) -> (86.783,13), heading 180 -> 0
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(132, 8),
                        new Pose(86.783, 13)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.95, () -> run(actions.launch3far()))
                .build();

        // === Path4: shooter -> stack 2 via curve ===
        // Blue: (57.217,15.315) -> (35.947,37.648) -> (14.464,35.734)
        // Mirrored: (86.783,15.315) -> (108.053,37.648) -> (129.536,35.734)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(86.783, 15.315),
                        new Pose(108.053, 37.648),
                        new Pose(129.536, 35.734)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path5: stack 2 -> shooter (line) ===
        // Blue: (14.464,35.734) -> (57.217,15.315)
        // Mirrored: (129.536,35.734) -> (86.783,15.315), heading 180 -> 0
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(129.536, 35.734),
                        new Pose(86.783, 15.315)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.9, () -> run(actions.launch3far()))
                .setGlobalDeceleration(0.45)
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // === Path1: drive to shooter, then WAIT, then LAUNCH ===
        PathChainTask path1Task = new PathChainTask(path1, 3)
                .addWaitAction(
                        0,
                        new SequentialAction(
                                new SleepAction(2),
                                actions.launch3faster()
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path1Task);

        // === First cycle: Path2 (out) + Path3 (back & shoot) ===
        addPath(path2, 0);
        addPath(sus, 0);
        addPath(sus2, 0);

        addPath(path3, 1);

        // === Second cycle: Path4 (curved out) + Path5 (back & shoot) ===
        addPath(path4, 0);
        addPath(path5, 1);

        // === Repeat Path2 + Path3 two more times ===
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
        // Not used in this auto
    }

    /**
     * Auto-aim turret based on robot pose and field target.
     */
    private void updateTurretAutoAim() {
        if (turret1 == null || turret2 == null || follower == null) return;

        Pose pose = follower.getPose();
        double currentX = pose.getX();
        double currentY = pose.getY();
        double heading  = pose.getHeading();

        double dx = targetX - currentX;
        double dy = targetY - currentY;

        double angleToTargetField = Math.atan2(dy, dx);

        double turretAngle = angleToTargetField - heading;

        while (turretAngle > Math.PI)  turretAngle -= 2 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

        double turretAngleDeg =
                Math.toDegrees(turretAngle) + turretTrimDeg + TURRET_CAL_OFFSET_DEG;

        double clampedAngle = Math.max(-turretMaxAngle,
                Math.min(turretMaxAngle, turretAngleDeg));

        double servoPosition;
        if (clampedAngle >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition
                    - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

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

