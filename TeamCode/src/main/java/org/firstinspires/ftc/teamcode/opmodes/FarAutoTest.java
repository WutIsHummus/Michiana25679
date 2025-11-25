package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.BezierPoint;
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

    // 🔒 Fixed turret angle (deg, relative to robot forward)
    // This is your OG tuned angle that works
    public static double FIXED_TURRET_ANGLE_DEG = -67.55;

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

        // Lock turret at fixed angle on init
        updateTurretFixed();

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

        // 🔒 Keep turret locked at fixed angle the entire auto
        updateTurretFixed();

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
        telemetry.addLine("=== TURRET (FIXED) ===");
        telemetry.addData("Fixed angle (deg)", FIXED_TURRET_ANGLE_DEG + turretTrimDeg);
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {
        // === Path1: (57.430, 8.934) -> (57.430, 15.315) ===
        // Shooter path to ~57,15 (no intake)
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57, 9),
                                new Pose(57, 15)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setGlobalDeceleration(0.6)
                .build();

        // === Path2: (57.430, 15.315) -> (10, 8) ===
        // Intake ON
        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57, 15),
                                new Pose(10, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Back to 57,15 → shooter path; stop intake before launch
        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10, 8),
                                new Pose(57, 15)))
                .setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(180))
                .setGlobalDeceleration(0.6)
                .addPath(
                        new BezierPoint(57, 15))
                .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(180))
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.9, () -> run(actions.stopIntake()))
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

        // === Path2: intake-only driving ===
        addPath(path2, 0);  // no extra wait, intake is already started via callbacks

        // === Path5: back to shooter point, then launch ===
        PathChainTask path5Task = new PathChainTask(path5, 1.0)
                .addWaitAction(
                        0,
                        actions.launch3slow()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path5Task);
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
     * 🔒 Fixed-angle turret control.
     * Uses the same angle→servo mapping as before, but
     * always holds FIXED_TURRET_ANGLE_DEG.
     */
    private void updateTurretFixed() {
        if (turret1 == null || turret2 == null) return;

        // Apply trim
        double desiredAngleDeg = FIXED_TURRET_ANGLE_DEG + turretTrimDeg;

        // Clamp to allowed turret range
        double clampedAngle = Math.max(-turretMaxAngle,
                Math.min(turretMaxAngle, desiredAngleDeg));

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

        // Apply backlash compensation to turret1, both shifted slightly like teleop
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
