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

    private PathChain path1, path3, path4, path5, path6, path7, path8, path9, path10, path11;

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

        follower.setStartingPose(new Pose(23.806, 126.000, Math.toRadians(145)));

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
                        new Pose(23.806, 126.000),
                        new Pose(87.484, 85.161),
                        new Pose(22.258, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22.258, 84.000),
                        new Pose(43.742, 99.871)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .setReversed(true)
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(43.742, 99.871),
                        new Pose(69.677, 54.968),
                        new Pose(21.677, 60.000)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(21.677, 60.000),
                        new Pose(32.516, 76.839),
                        new Pose(43.742, 99.871)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .setReversed(true)
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(43.742, 99.871),
                        new Pose(57.097, 75.290),
                        new Pose(15.677, 70.065)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(270))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(15.677, 70.065),
                        new Pose(85.548, 33.290),
                        new Pose(21.290, 35.419)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(21.290, 35.419),
                        new Pose(43.548, 99.677)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(43.548, 99.677),
                        new Pose(11.032, 50.129),
                        new Pose(13.161, 10.452)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(270))
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(13.161, 10.452),
                        new Pose(47.419, 32.323),
                        new Pose(43.742, 99.871)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(130))
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(43.742, 99.871),
                        new Pose(10.839, 50.129),
                        new Pose(13.161, 10.452)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(270))
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
        addPath(path11, 0);
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
