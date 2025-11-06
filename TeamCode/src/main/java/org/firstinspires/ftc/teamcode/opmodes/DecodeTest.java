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

@Autonomous(name = "DecodeTest")
public class DecodeTest extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotor shootr, shootl;
    private Servo reargate, launchgate;

    private RobotActions actions;

    private PathChain path1, path2, path3;

    @Override
    public void init() {
        pathTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotor.class, "intakeback");
        shootr      = hardwareMap.get(DcMotor.class, "shootr");
        shootl      = hardwareMap.get(DcMotor.class, "shootl");

        reargate   = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        actions = new RobotActions(intakefront, intakeback, shootr, shootl, launchgate, reargate);

        // Starting position
        follower.setStartingPose(new Pose(24.0, 126.0, Math.toRadians(144)));

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
        // === Path 1 ===
        // Start (24,126) → (49,96)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(24.0, 126.0),
                        new Pose(49.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        // === Path 2 ===
        // (49,96) → (16,84) with control (52,84)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(49.0, 96.0),
                        new Pose(52.0, 84.0),
                        new Pose(16.0, 84.0)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // === Path 3 ===
        // (16,84) → (49,96)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(16.0, 84.0),
                        new Pose(49.0, 96.0)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // addPath(path, waitSeconds)
        addPath(path1, 1.0);
        addPath(path2, 1.0);
        addPath(path3, 1.0);
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
