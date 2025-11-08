package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "ShortShootAuto")
public class ShortShootAuto extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;

    private RobotActions actions;

    private PathChain path1, path2;

    @Override
    public void init() {
        pathTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback  = hardwareMap.get(DcMotor.class, "intakeback");
        shootr      = hardwareMap.get(DcMotorEx.class, "shootr");
        shootl      = hardwareMap.get(DcMotorEx.class, "shootl");
        hood1       = hardwareMap.get(Servo.class, "hood 1");
        turret1     = hardwareMap.get(Servo.class, "turret1");
        turret2     = hardwareMap.get(Servo.class, "turret2");
        reargate    = hardwareMap.get(Servo.class, "reargate");
        launchgate  = hardwareMap.get(Servo.class, "launchgate");

        // Shooter motor directions
        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Neutral servo positions
        hood1.setPosition(0.54);
        turret1.setPosition(0.51);
        turret2.setPosition(0.51);

        actions = new RobotActions(
                intakefront, intakeback,
                shootr, shootl,
                launchgate, reargate,
                turret1, turret2,
                hood1
        );

        // Start pose (same as your other autos)
        follower.setStartingPose(new Pose(57.004431, 9.146233, Math.toRadians(90)));

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

        // Hold shooter at 900 RPM
        run(actions.holdShooterAtRPMclose(1350, 30));

        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {
        // Path 1: Start -> (26.5878877, 128.898079) | heading 143° -> 143°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(26.5878877, 128.898079),
                        new Pose(25.5878877, 127.898079)))
                .build();

        // Path 2: (26.5878877, 128.898079) -> (30.2038404, 113.1580502) | 143° -> 143°
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(26.5878877, 128.898079),
                        new Pose(30.2038404, 113.1580502)))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Path 1 task: run path, then wait 20 seconds, then shoot
        PathChainTask path1Task = new PathChainTask(path1, 20.0) // 20-second wait AFTER path
                .addWaitAction(
                        () -> true,
                        new SequentialAction(
                                actions.launch3()   // fire 3 at 900 RPM
                        )
                )
                .setMaxWaitTime(25.0)  // safety cap
                .setWaitCondition(() -> true);

        tasks.add(path1Task);

        // Then just drive Path 2 (no extra wait / shooting)
        addPath(path2, 0);
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
    protected void startTurn(TurnTask task) {
        // Not used
    }
}
