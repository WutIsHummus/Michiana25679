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

@Autonomous(name = "1 - redfar21nospike")
public class Farred21 extends PathChainAutoOpMode {

    private static final double FIELD_SIZE = 144.0;

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

    // Mirrored sign (was tuned on blue with heading 180; red mirror flips turret sign)
    public static double FIXED_TURRET_ANGLE_DEG = +68.55;

    // === Field target (RED) ===
    // Blue used mirrorX(128) -> 16. Red is the original.
    public static double targetX = 132;
    public static double targetY = 125.0;

    // Mirrored sign (blue offset applied to turret angle; mirror flips sign)
    public static double TURRET_CAL_OFFSET_DEG = +1.5;

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

        // Start pose matches Path1 start (MIRRORED from blue)
        // Blue start: (57.430, 8.934, 180°)
        // Red mirror: (86.570, 8.934, 0°)
        follower.setStartingPose(new Pose(
                mirrorX(57.430), 8.934,
                mirrorHeading(Math.toRadians(180))
        ));

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
        // Mirrored version of the BLUE auto:
        // mirrorX(x) = 144 - x
        // mirrorHeading(theta) = PI - theta (normalized)

        // === Path1: (57.430, 8.934) -> (57.217, 15.315) ===
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(mirrorX(57.430), 8.934),
                                new Pose(mirrorX(57.217), 15.315)))
                .setConstantHeadingInterpolation(mirrorHeading(Math.toRadians(180)))
                .setGlobalDeceleration(0.5)
                .build();

        // === Path2: (57.217, 15.315) -> (12, 8) ===
        path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mirrorX(57.217), 15.315),
                                new Pose(mirrorX(40.000),  10.000),
                                new Pose(mirrorX(12.000),   8.000)))
                .setLinearHeadingInterpolation(
                        mirrorHeading(Math.toRadians(180)),
                        mirrorHeading(Math.toRadians(180)))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        sus = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(mirrorX(12.000), 8.000),
                                new Pose(mirrorX(15.000), 9.000)))
                .setLinearHeadingInterpolation(
                        mirrorHeading(Math.toRadians(170)),
                        mirrorHeading(Math.toRadians(190)))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        sus2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(mirrorX(15.000), 9.000),
                                new Pose(mirrorX(12.000), 8.000)))
                .setLinearHeadingInterpolation(
                        mirrorHeading(Math.toRadians(190)),
                        mirrorHeading(Math.toRadians(180)))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path3: (12, 8) -> (57.217, 13) ===
        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(mirrorX(12.000), 8.000),
                                new Pose(mirrorX(57.217), 13.000)))
                .setConstantHeadingInterpolation(mirrorHeading(Math.toRadians(180)))
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.9, () -> run(actions.launch3far()))
                .build();

        // === Path4: shooter -> stack 2 via curve ===
        path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mirrorX(57.217), 15.315),
                                new Pose(mirrorX(35.947), 37.648),
                                new Pose(mirrorX(14.464), 35.734)
                        )
                )
                // Tangent heading interpolation will follow the mirrored curve tangents naturally
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // === Path5: stack 2 -> shooter (line) ===
        path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(mirrorX(14.464), 35.734),
                                new Pose(mirrorX(57.217), 15.315)
                        )
                )
                .setConstantHeadingInterpolation(mirrorHeading(Math.toRadians(180)))
                .addParametricCallback(0.9, () -> run(actions.launch3far()))
                .setGlobalDeceleration(0.45)
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // === Path1: drive to shooter, then WAIT, then LAUNCH ===
        PathChainTask path1Task = new PathChainTask(path1, 4)
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

        // === Repeat Path2 + Path3 (kept exactly as your original list) ===
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

    // -----------------------------
    // Mirroring helpers (field X flip)
    // -----------------------------

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double mirrorX(double x) {
        return FIELD_SIZE - x;
    }

    // Mirror heading across vertical axis: theta' = PI - theta
    private static double mirrorHeading(double headingRad) {
        return normalizeRadians(Math.PI - headingRad);
    }

    @Override
    public void stop() {
        try {
            PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}
