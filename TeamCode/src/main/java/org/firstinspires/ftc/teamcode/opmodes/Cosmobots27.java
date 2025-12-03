package org.firstinspires.ftc.teamcode.opmodes;

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

@Autonomous(name = "27CBclose")
public class Cosmobots27 extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    // --- Turret + goal constants (same as Blue teleop / Faster18ball) ---

    // Field target for turret aim (mirrored blue-side coords)
    public static double targetX = 144.0 - 128.0; // 16.0
    public static double targetY = 125.0;

    // Goal “zone” center (for distance telemetry if you want)
    public static double goalZoneX = 144.0 - 116.0; // 28.0
    public static double goalZoneY = 116.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 135;    // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.02;

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
        hood1.setPosition(0.48);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);

        updateTurretFromPose();

        actions = new RobotActions(
                intakefront, intakeback,
                shootr, shootl,
                launchgate, reargate,
                turret1, turret2,
                hood1
        );

        // Start pose = Path1 start
        // Heading is approximate; you can tweak later if needed.
        follower.setStartingPose(new Pose(
                22.759,
                125.920,
                Math.toRadians(143) // roughly along Path1 direction
        ));

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

        // Auto-aim turret at goal from current pose
        updateTurretFromPose();

        // keep shooter spun up (tune RPM as needed)
        run(actions.holdShooterAtRPMclose(1350, 30));

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
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {
        // These are your new 9 paths, wired into the existing pattern
        // Shooter paths (return to speaker): 1,4,7,9
        // Intake sweeps: 2,3,5,6,8

        // Path1: preload -> first shooter position
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.759, 125.920),
                                new Pose(53.388, 89.761)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                // Fire 3 near the end of the path
                .addParametricCallback(0.8, () -> run(actions.launch3faster()))
                .build();

        // Path2: sweep front row (intake on)
        path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.388, 89.761),
                                new Pose(52.538, 81.891),
                                new Pose(16, 83.167)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path3: small wiggle to grab more
        path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16, 83.167),
                                new Pose(26.375, 78.700),
                                new Pose(15.953, 74)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path4: return to shooter spot
        path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.953, 74),
                                new Pose(53.176, 89.761)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.8, () -> run(actions.launch3faster()))
                .build();

        // Path5: longer sweep downfield
        path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.176, 89.761),
                                new Pose(57.855, 58.281),
                                new Pose(9.359, 52)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path6: small follow-up wiggle intake
        path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.359, 52),
                                new Pose(23.823, 61.684),
                                new Pose(15.953, 60)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path7: back to shooter
        path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.953, 60),
                                new Pose(53.176, 89.335)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.8, () -> run(actions.launch3faster()))
                .build();

        // Path8: big sweep across field
        path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.176, 89.335),
                                new Pose(15, 55),
                                new Pose(13, 45),
                                new Pose(9, 10)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path9: final return to shooter-ish position
        path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(6.806, 24.248),
                                new Pose(61.684, 101.247)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.8, () -> run(actions.launch3faster()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // 1) Preload: Path1 → shoot
        PathChainTask path1Task = new PathChainTask(path1, 0.7);
        tasks.add(path1Task);

        // 2) First sweep: Path2 + Path3 (intake only)
        addPath(path2, 0.0);
        addPath(path3, 0.0);

        // 3) Return & shoot: Path4
        PathChainTask path4Task = new PathChainTask(path4, 0.7);
        tasks.add(path4Task);

        // 4) Second sweep: Path5 + Path6
        addPath(path5, 0.0);
        addPath(path6, 0.0);

        // 5) Return & shoot: Path7
        PathChainTask path7Task = new PathChainTask(path7, 0.7);
        tasks.add(path7Task);

        // 6) Final big sweep: Path8
        addPath(path8, 0.0);

        // 7) Final return & shoot: Path9
        PathChainTask path9Task = new PathChainTask(path9, 0.9);
        tasks.add(path9Task);
        addPath(path8, 0.0);

        tasks.add(path9Task);
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
     * Uses Pedro pose to:
     *  - compute angle to goal
     *  - compute distance to goal
     *  - set turret1/turret2 servo positions accordingly
     */
    private void updateTurretFromPose() {
        if (follower == null || turret1 == null || turret2 == null) return;

        // Current robot pose from Pedro (Pinpoint)
        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading(); // radians

        // Vector from robot to turret target
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        // Distance to goal (inches)
        double distanceToGoalInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Field angle to target
        double angleToTargetField = Math.atan2(deltaY, deltaX);

        // Angle of target relative to robot heading (turret rotation)
        double turretAngle = angleToTargetField - currentHeading;

        // Normalize to [-PI, PI]
        while (turretAngle > Math.PI)  turretAngle -= 2.0 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2.0 * Math.PI;

        // Convert to degrees and apply trim
        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;

        // Clamp to allowed turret range
        double clampedAngle = Math.max(
                -turretMaxAngle,
                Math.min(turretMaxAngle, turretAngleDegrees)
        );

        // Map angle → servo position
        double servoPosition;
        if (clampedAngle >= 0) {
            // Positive angle = turn right
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            // Negative angle = turn left
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        // Clamp servo range
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // Apply backlash compensation to turret1
        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        // Optional turret telemetry (shows in init/loop)
        telemetry.addData("Turret dist (in)", "%.1f", distanceToGoalInches);
        telemetry.addData("Turret angle (deg)", "%.1f", turretAngleDegrees);
        telemetry.addData("Turret servo", "%.3f", servoPosition);
    }

    @Override
    public void stop() {
        try {
            PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}
