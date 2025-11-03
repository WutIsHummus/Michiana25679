package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "decodetest")
public class DecodeTest extends PathChainAutoOpMode {

    private Follower follower;
    
    private DcMotor intakefront, intakeback;
    private DcMotor shootr, shootl;
    private Servo reargate, launchgate;
    
    private RobotActions actions;

    private PathChain path1, path3, path4, path5, path6, path7, path8, path9, path10;

    @Override
    public void init() {
        pathTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        
        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback = hardwareMap.get(DcMotor.class, "intakeback");
        shootr = hardwareMap.get(DcMotor.class, "shootr");
        shootl = hardwareMap.get(DcMotor.class, "shootl");
        
        reargate = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        
        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        
        actions = new RobotActions(intakefront, intakeback, shootr, shootl, launchgate, reargate);

        follower.setStartingPose(new Pose(19.548, 118.452, Math.toRadians(145)));

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
        runTasks();

        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("PathActive", isPathActive());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(19.548, 118.452),
                        new Pose(94, 78),
                        new Pose(19.548, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.safePositions()))
                .addParametricCallback(0.5, () -> run(actions.startIntake()))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(19.548, 84.000),
                        new Pose(55.935, 88.258)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .setReversed(true)
                .addParametricCallback(0, () -> run(actions.stopIntake()))
                .addParametricCallback(0.3, () -> run(actions.intakeAndLaunch()))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.935, 88.258),
                        new Pose(62.323, 55.742),
                        new Pose(21.484, 59.613)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.startIntake()))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(21.484, 59.613),
                        new Pose(30.387, 68.903),
                        new Pose(55.355, 88.258)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .setReversed(true)
                .addParametricCallback(0, () -> run(actions.stopIntake()))
                .addParametricCallback(0.3, () -> run(actions.intakeAndLaunch()))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.355, 88.258),
                        new Pose(66.194, 29.419),
                        new Pose(20.903, 35.226)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .addParametricCallback(0.5, () -> run(actions.startIntake()))
                .build();

        // Path7
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(20.903, 35.226),
                        new Pose(50.323, 76.452),
                        new Pose(17.419, 70.258)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(17.419, 70.258),
                        new Pose(55.548, 88.065)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .addParametricCallback(0, () -> run(actions.stopIntake()))
                .addParametricCallback(0.3, () -> run(actions.intakeAndLaunch()))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.548, 88.065),
                        new Pose(11.032, 50.129),
                        new Pose(14.700, 6.581)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(270))
                .addParametricCallback(0.5, () -> run(actions.startIntake()))
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(9.677, 6.581),
                        new Pose(47.419, 32.323),
                        new Pose(55.355, 88.258)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(130))
                .addParametricCallback(0, () -> run(actions.stopIntake()))
                .addParametricCallback(0.3, () -> run(actions.intakeAndLaunch()))
                .addParametricCallback(1, () -> run(actions.safePositions()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();
        addPath(path1, 0);
        addPath(path3, 1);
        addPath(path4, 0);
        addPath(path5, 1);
        addPath(path6, 0);
        addPath(path7, 0.5);
        addPath(path8, 1);
        addPath(path9, 0);
        addPath(path10, 1);
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
        // Not used
    }
}
