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

@Autonomous(name = "Test15ball")
public class Test15ball extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10;

    @Override
    public void init() {
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
        turret1.setPosition(0.29);
        turret2.setPosition(0.265);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.5);

        actions = new RobotActions(intakefront, intakeback, shootr, shootl,
                launchgate, reargate, turret1, turret2, hood1);

        // Start pose matches Path1 start
        follower.setStartingPose(new Pose(56.0, 8.0, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        currentTaskIndex = 0;
        taskPhase        = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

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
        // Path1: (56,8) -> (56.579,87.421), heading 270 -> 230
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(56.579, 87.421)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(230))
                .build();

        // Path2: (56.579,87.421) -> (15.315,85.719), heading 230 -> 180 (INTAKE)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.421),
                        new Pose(15.315, 85.719)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path3: (15.315,85.719) -> (56.792,87.421), heading 180 -> 230 (STOP INTAKE)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15.315, 85.719),
                        new Pose(56.792, 87.421)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path4: (56.792,87.421) -> (14.889,64.874) via (41.903,58.919), heading 230 -> 180 (INTAKE)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.792, 87.421),
                        new Pose(41.903, 58.919),
                        new Pose(11, 62)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(150))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path5: (14.889,64.874) -> (56.792,87.634), heading 180 -> 230 (STOP INTAKE)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(11, 62),
                        new Pose(24, 60),
                        new Pose(56.792, 87.634)))
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(230))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path6: (56.792,87.634) -> (15.953,35.734) via (54.665,35.734), heading 230 -> 180 (INTAKE)
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.792, 87.634),
                        new Pose(54.665, 35.734),
                        new Pose(15.953, 35.734)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path7: (15.953,35.734) -> (56.579,87.634), heading 180 -> 230 (STOP INTAKE)
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15.953, 35.734),
                        new Pose(56.579, 87.634)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path8: (56.579,87.634) -> (11.486,11.273), heading 230 -> 250 (INTAKE)
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.634),
                        new Pose(11.486, 11.273)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(250))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path9: (11.486,11.273) -> (56.579,87.634), heading 250 -> 230 (STOP INTAKE)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(11.486, 11.273),
                        new Pose(56.579, 87.634)))
                .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(230))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path10: (56.579,87.634) -> (46.157,76.360), heading 230 -> 230 (JUST DRIVE)
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
        PathChainTask path1Task = new PathChainTask(path1, 1.5)
                .addWaitAction(
                        0,
                        new SequentialAction(
                                actions.launch3()
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path1Task);

        // Path2: intake only
        addPath(path2, 0);

        // Shoot after Path3
        PathChainTask path3Task = new PathChainTask(path3, 1.5)
                .addWaitAction(
                        0,
                        new SequentialAction(
                                actions.launch3()
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path3Task);

        // Path4: intake only
        addPath(path4, 0);

        // Shoot after Path5
        PathChainTask path5Task = new PathChainTask(path5, 1.5)
                .addWaitAction(
                        0,
                        new SequentialAction(
                                actions.launch3()
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path5Task);

        // Path6: intake only
        addPath(path6, 0);

        // Shoot after Path7
        PathChainTask path7Task = new PathChainTask(path7, 1.5)
                .addWaitAction(
                        0,
                        new SequentialAction(
                                actions.launch3()
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path7Task);

        // Path8: intake only
        addPath(path8, 0);

        // Shoot after Path9
        PathChainTask path9Task = new PathChainTask(path9, 1.5)
                .addWaitAction(
                        0,
                        new SequentialAction(
                                actions.launch3()
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path9Task);

        // Path10: final drive, no actions
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
        follower.followPath((PathChain) task.pathChain, true);
    }

    @Override
    protected void startTurn(TurnTask task) {
        // Not used in this auto
    }

    @Override
    public void stop() {
        try {
            org.firstinspires.ftc.teamcode.opmodes.PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}
