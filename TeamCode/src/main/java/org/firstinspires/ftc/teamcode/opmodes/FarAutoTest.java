package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "farautotest")
public class FarAutoTest extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5;

    // --- Turret constants ---

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 140;    // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.027;

    // This is the OG tuned angle when robot pose = (57.217,15.315, heading 180°)
    // and facing the right/backdrop.
    public static double FIXED_TURRET_ANGLE_DEG = -68.55;

    // === Field target (BLUE) ===
    // Same as your Blue TeleOp:
    // Red-side 128,125 mirrored across field width 144 → (16,125)
    public static double targetX = 144.0 - 128.0;  // 16.0
    public static double targetY = 125.0;

    // Small calibration offset so that at the shooter pose
    // the field-based angle matches the empirically tuned -68.55°.
    // Geometry at (57.217,15.315, heading 180) gives about -69.40°,
    // so we add +0.85°.
    public static double TURRET_CAL_OFFSET_DEG = -1.5;

    @Override
    public void init() {
        super.init();  // important so PathChainAutoOpMode can init its state

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

        actions = new RobotActions(intakefront, intakeback, shootr, shootl,
                launchgate, reargate, turret1, turret2, hood1);

        // Start pose matches Path1 start
        follower.setStartingPose(new Pose(57.430, 8.934, Math.toRadians(180)));

        // ⭕ Turret now uses auto-aim from pose

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();  // important so task state is reset correctly
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // ⭕ Continuously auto-aim turret to target using localization
        updateTurretAutoAim();

        // keep shooter spun up (tune as needed)
        run(actions.holdShooterAtRPMclose(1750, 30));

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

        // For debug: show current pose and computed turret angle
        Pose pose = follower.getPose();
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Target", "(%.1f, %.1f)", targetX, targetY);

        // Recompute angle for telemetry
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
        // Use the precise shooter pose everywhere: (57.217, 15.315)

        // === Path1: (57.430, 8.934) -> (57.217, 15.315) ===
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.430, 8.934),
                                new Pose(57.217, 15.315)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setGlobalDeceleration(0.5)
                .build();

        // === Path2: (57.217, 15.315) -> (10, 8) ===
        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.217, 15.315),
                                new Pose(10, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path3: (10, 8) -> (57.217, 15.315) ===
        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10, 8),
                                new Pose(57.217, 15.315)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setGlobalDeceleration(0.5)
                .addParametricCallback(0.9, () -> run(actions.stopIntake()))
                .build();

        // === Path4: shooter -> stack 2 via curve ===
        path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.217, 15.315),
                                new Pose(35.947, 37.648),
                                new Pose(14.464, 35.734)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path5: stack 2 -> shooter (line) ===
        path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.464, 35.734),
                                new Pose(57.217, 15.315)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.9, () -> run(actions.stopIntake()))
                .setGlobalDeceleration(0.5)
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
                                new SleepAction(1),    // wait a bit
                                actions.launch3slow()  // then fire 3
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path1Task);

        // === First cycle: Path2 (out) + Path3 (back & shoot) ===
        addPath(path2, 0);  // intake-only drive

        PathChainTask path3Task = new PathChainTask(path3, 1.0)
                .addWaitAction(
                        0,
                        actions.launch3slow()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path3Task);

        // === Second cycle: Path4 (curved out) + Path5 (back & shoot) ===
        addPath(path4, 0);  // intake-only curve

        PathChainTask path5Task = new PathChainTask(path5, 1.0)
                .addWaitAction(
                        0,
                        actions.launch3slow()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path5Task);

        // === Repeat Path2 + Path3 two more times ===

        // Repeat 1
        addPath(path2, 0);
        PathChainTask path3TaskRepeat1 = new PathChainTask(path3, 1.0)
                .addWaitAction(
                        0,
                        actions.launch3slow()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path3TaskRepeat1);

        // Repeat 2
        addPath(path2, 0);
        PathChainTask path3TaskRepeat2 = new PathChainTask(path3, 1.0)
                .addWaitAction(
                        0,
                        actions.launch3slow()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path3TaskRepeat2);
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
     * Uses the same math as TeleOp:
     *  1) Compute angle to target in field frame.
     *  2) Subtract robot heading to get turret angle.
     *  3) Normalize to [-180,180], apply trim + calibration.
     *  4) Convert angle → servo positions (with backlash on turret1).
     */
    private void updateTurretAutoAim() {
        if (turret1 == null || turret2 == null || follower == null) return;

        Pose pose = follower.getPose();
        double currentX = pose.getX();
        double currentY = pose.getY();
        double heading  = pose.getHeading();

        // Vector from robot → target (field coordinates)
        double dx = targetX - currentX;
        double dy = targetY - currentY;

        // Field-frame angle to target
        double angleToTargetField = Math.atan2(dy, dx);

        // Turret angle relative to robot forward
        double turretAngle = angleToTargetField - heading;

        // Normalize to [-PI, PI]
        while (turretAngle > Math.PI)  turretAngle -= 2 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

        // Convert to degrees & apply trim + calibration offset
        double turretAngleDeg =
                Math.toDegrees(turretAngle) + turretTrimDeg + TURRET_CAL_OFFSET_DEG;

        // Clamp to mechanical range
        double clampedAngle = Math.max(-turretMaxAngle,
                Math.min(turretMaxAngle, turretAngleDeg));

        // Map angle → servo position
        double servoPosition;
        if (clampedAngle >= 0) {
            // Positive angle = turn right
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            // Negative angle = turn left
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition
                    - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        // Clamp servo range
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // Backlash compensation on turret1, plus small shift like TeleOp
        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);
    }

    @Override
    public void stop() {
        try {
            org.firstinspires.ftc.teamcode.opmodes.PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}
