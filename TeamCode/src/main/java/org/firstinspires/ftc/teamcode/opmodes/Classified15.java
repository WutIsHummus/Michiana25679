package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "15 classified")
public class Classified15 extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12;
    // --- Turret + goal constants (copied from Blue teleop) ---

    // Field target for turret aim (mirrored blue-side coords)
    public static double targetX = 144.0 - 128.0; // 16.0
    public static double targetY = 125.0;

    // Goal “zone” center (for distance telemetry if you want)
    public static double goalZoneX = 144.0 - 116.0; // 28.0
    public static double goalZoneY = 116.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;  // max left
    public static double turretRightPosition  = 0.85;  // max right
    public static double turretMaxAngle       = 140;   // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

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
        hood1.setPosition(0.48);
        turret1.setPosition(0.275);
        turret2.setPosition(0.25);
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

        // Start pose matches Path1 start
        follower.setStartingPose(new Pose(56.0, 8.0, Math.toRadians(270)));

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
        // Aim turret based on current pose and goal
        updateTurretFromPose();

        // keep shooter spun up (tune as needed)
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
        // Shooter heading we want to hold at the end
        double shooterHeading = Math.toRadians(215);

        // === Path1: (56,8) -> (56.579,87.421), then settle at shooter pose ===
        path1 = follower.pathBuilder()
                // Main drive segment, tangent heading but reversed so it drives backwards
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(56.579, 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                // Final settle as a BezierPoint at the same shooter pose with explicit heading
//                .addPath(new BezierPoint(
//                        new Pose(56.579, 87.421, shooterHeading)))
//                .setConstantHeadingInterpolation(230)
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        // === Path2: (56.579,87.421) -> (20,82), intake path (not a shooter path) ===
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.421),
                        new Pose(20, 82)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path3: (20,82) -> (56.792,87.421), then settle at shooter pose ===
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20, 82),
                        new Pose(56.792, 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)

//                .addPath(new BezierPoint(
//                        new Pose(56.792, 87.421, shooterHeading)))
//                .setConstantHeadingInterpolation(shooterHeading)
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        // === Path4: (56.792,87.421) -> curve to (14,52), intake path ===
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.792, 87.421),
                        new Pose(41.903, 58.919),
                        new Pose(14, 52)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(150))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path5: (14,52) curve -> (56.792,87.634), then settle at shooter pose ===
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(14, 52),
                        new Pose(24, 50),
                        new Pose(56.792, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)

//                .addPath(new BezierPoint(
//                        new Pose(56.792, 87.634, shooterHeading)))
//                .setConstantHeadingInterpolation(shooterHeading)
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.7, () -> run(actions.stopIntake()))

                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        // === Path6: (56.792,87.634) curve -> (10,28), intake path ===
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.792, 87.634),
                        new Pose(54.665, 35.734),
                        new Pose(10, 28)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path7: (10,28) -> (56.579,87.634), then settle at shooter pose ===
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10, 28),
                        new Pose(56.579, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
//                .addPath(new BezierPoint(
//                        new Pose(56.579, 87.634, shooterHeading)))
//                .setConstantHeadingInterpolation(shooterHeading)
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.7, () -> run(actions.stopIntake()))
                .addParametricCallback(0.95, () -> run(actions.launch3faster()))

                .build();

        // === Path8: (56.579,87.634) -> (11.486,11.273), intake path ===
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.634),
                        new Pose(11.486, 11.273)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(250))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path9: (11.486,11.273) -> (56.579,87.634), then settle at shooter pose ===
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(11.486, 11.273),
                        new Pose(56.579, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
//                .addPath(new BezierPoint(
//                        new Pose(56.579, 87.634, shooterHeading)))
//                .setConstantHeadingInterpolation(shooterHeading)
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.9, () -> run(actions.stopIntake()))
                .addParametricCallback(0.95, () -> run(actions.launch3faster()))

                .build();

        // === Path11: (56.579,87.634) -> (11.486,11.273), intake path ===
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.634),
                        new Pose(11.486, 11.273)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(250))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path12: (11.486,11.273) -> (56.579,87.634), then settle at shooter pose ===
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(11.486, 11.273),
                        new Pose(56.579, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
//                .addPath(new BezierPoint(
//                        new Pose(56.579, 87.634, shooterHeading)))
//                .setConstantHeadingInterpolation(shooterHeading)
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.9, () -> run(actions.stopIntake()))
                .addParametricCallback(0.95, () -> run(actions.launch3faster()))

                .build();

        // === Path10: (56.579,87.634) -> (46.157,76.360), simple reposition ===
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.634),
                        new Pose(46.157, 76.360)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(230))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Shoot after Path1 (preload)
        PathChainTask path1Task = new PathChainTask(path1, 1);
        tasks.add(path1Task);

        // Path2: intake only
        addPath(path2, 0);

        // Shoot after Path3
        PathChainTask path3Task = new PathChainTask(path3, 1);
        tasks.add(path3Task);

        // Path4: intake only
        addPath(path4, 0);

        // Shoot after Path5
        PathChainTask path5Task = new PathChainTask(path5, 1)
                .addWaitAction(
                        0,
                        actions.launch3faster()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path5Task);

        // Path6: intake only
        addPath(path6, 0);

        // Shoot after Path7
        PathChainTask path7Task = new PathChainTask(path7, 1)
                .addWaitAction(
                        0,
                        actions.launch3faster()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path7Task);

        // Path8: intake only
        addPath(path8, 0);

        // Shoot after Path9
        PathChainTask path9Task = new PathChainTask(path9, 1)
                .addWaitAction(
                        0,
                        actions.launch3faster()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path9Task);

        // Path11: intake only
        addPath(path11, 0);

        // Shoot after Path12
        PathChainTask path12Task = new PathChainTask(path12, 1)
                .addWaitAction(
                        0,
                        actions.launch3faster()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path12Task);

        // Final reposition
        addPath(path10, 0);
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

        // Distance to goal (inches, since your field coords are in inches)
        double distanceToGoalInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Field angle to target (0 rad = +X, CCW positive)
        double angleToTargetField = Math.atan2(deltaY, deltaX);

        // Angle of target relative to robot heading (what turret must turn)
        double turretAngle = angleToTargetField - currentHeading;

        // Normalize to [-PI, PI]
        while (turretAngle > Math.PI)  turretAngle -= 2.0 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2.0 * Math.PI;

        // Convert to degrees and apply trim
        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;

        // Clamp to allowed turret range
        double clampedAngle = Math.max(-turretMaxAngle,
                Math.min(turretMaxAngle, turretAngleDegrees));

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

        // Optional: telemetry so you can see what's happening in auto
        telemetry.addData("Turret dist (in)", "%.1f", distanceToGoalInches);
        telemetry.addData("Turret angle (deg)", "%.1f", turretAngleDegrees);
        telemetry.addData("Turret servo", "%.3f", servoPosition);
    }


    @Override
    public void stop() {
        try {
            org.firstinspires.ftc.teamcode.opmodes.PoseStore.saveBlue(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}

