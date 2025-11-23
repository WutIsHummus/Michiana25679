package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
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

@Autonomous(name = "FarRedShootFarther")
public class FarRedShootFarther extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;

    @Override
    public void init() {
        pathTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotor.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");
        hood1 = hardwareMap.get(Servo.class, "hood 1");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        reargate   = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");

        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        hood1.setPosition(0.54);
        turret1.setPosition(0.51);
        turret2.setPosition(0.51);

        actions = new RobotActions(intakefront, intakeback, shootr, shootl, launchgate, reargate, turret1, turret2, hood1);

        // Mirror start: (144 - 57.004431 = 86.995569)
        follower.setStartingPose(new Pose(86.995569, 9.146233, Math.toRadians(90)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        run(actions.holdShooterAtRPMclose(1400, 30));
        runTasks();

        double shooterRPM = actions.shooter.getCurrentRPM();
        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());
        telemetry.addLine("=== SHOOTER DEBUG ===");
        telemetry.addData("Shooter RPM (avg)", "%.0f", shooterRPM);
        telemetry.addData("Right RPM", "%.0f", rpmR);
        telemetry.addData("Left RPM", "%.0f", rpmL);
        telemetry.addData("Right TPS", "%.1f", vR);
        telemetry.addData("Left TPS", "%.1f", vL);
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // Path 1
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(86.995569, 9.146233),
                        new Pose(95.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(46))
                .build();

        // Path 2
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(95.0, 96.0),
                        new Pose(62.322009, 79.976366),
                        new Pose(123.0, 83.592319)))
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.0, 83.592319),
                        new Pose(95.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 4
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(95.0, 96.0),
                        new Pose(70.61743, 55.090103),
                        new Pose(133.0, 55.515510)))
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 6 (was 5 skipped)
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130.0, 55.515510),
                        new Pose(95.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 7
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(95.0, 96.0),
                        new Pose(51.048744, 31.054653),
                        new Pose(134.002954, 34.883309)))
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 8
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(134.002954, 34.883309),
                        new Pose(95.29099, 95.929099)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 9
        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(95.29099, 95.929099),
                        new Pose(124.856721, 53.388479),
                        new Pose(133.577548, 10.209749)))
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(270))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 10
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(133.577548, 10.209749),
                        new Pose(95.078287, 95.929099)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(46))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 11 final park
        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(105.713414, 105.075332),
                        new Pose(92.0, 120.0)))
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(46))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Shoot after Path 1
        PathChainTask path1Task = new PathChainTask(path1, 1.5)
                .addWaitAction(() -> true, new SequentialAction(actions.launch3()))
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path1Task);

        addPath(path2, 0);

        // Shoot after Path 3
        PathChainTask path3Task = new PathChainTask(path3, 1.5)
                .addWaitAction(() -> true, new SequentialAction(actions.launch3()))
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path3Task);

        addPath(path4, 0);

        // Shoot after Path 6
        PathChainTask path6Task = new PathChainTask(path6, 1.5)
                .addWaitAction(() -> true, new SequentialAction(actions.launch3()))
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path6Task);

        addPath(path7, 0);

        // Shoot after Path 8
        PathChainTask path8Task = new PathChainTask(path8, 1.5)
                .addWaitAction(() -> true, new SequentialAction(actions.launch3()))
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path8Task);

        addPath(path9, 0);

        // Shoot after Path 10
        PathChainTask path10Task = new PathChainTask(path10, 1.5)
                .addWaitAction(() -> true, new SequentialAction(actions.launch3()))
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path10Task);

        addPath(path11, 0);
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
    public void stop() {
        try {
            // follower.getPose() returns Pedro Pose
            org.firstinspires.ftc.teamcode.opmodes.PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
    @Override
    protected void startTurn(TurnTask task) {
        // Not used
    }
}
