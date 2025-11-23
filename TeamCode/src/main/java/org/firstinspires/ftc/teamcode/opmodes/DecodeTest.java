package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

@Autonomous(name = "DecodeTest")
public class DecodeTest extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate;

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

        reargate   = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");
        
        // Set motor directions
        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        actions = new RobotActions(intakefront, intakeback, shootr, shootl, launchgate, reargate);
        // Preferred hood set; threeBallabsoluteclose() will set hood for close as needed
        // Keep this if you want a default hood on init:
        // actions.hood.setPosition(0.49);

        // Start Point from the canvas: (24.460856, 125.920236) with heading ~142°
        follower.setStartingPose(new Pose(24.460856, 125.920236, Math.toRadians(144)));

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
        run(actions.holdShooterAtRPMclose(1400,30));
        runTasks();

        // Shooter RPM telemetry (requires getCurrentRPM() in Shooter class)
        double shooterRPM = actions.shooter.getCurrentRPM();
        
        // Debug: Show individual motor velocities
        double vR = shootr.getVelocity();
        double vL = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;

        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("PathActive", isPathActive());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("", "");
        telemetry.addLine("=== SHOOTER DEBUG ===");
        telemetry.addData("Shooter RPM (avg)", "%.0f", shooterRPM);
        telemetry.addData("Right Motor RPM", "%.0f", rpmR);
        telemetry.addData("Left Motor RPM", "%.0f", rpmL);
        telemetry.addData("Right Motor TPS (raw)", "%.1f", vR);
        telemetry.addData("Left Motor TPS (raw)", "%.1f", vL);
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {
        // Path 1: (24.460856,125.920236) -> (49,96)  | heading 144° -> 134°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(24.460856, 125.920236),
                        new Pose(49.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(134))
                // Spin up for 39 in (3.25 ft) at the start of the first path
                //.addParametricCallback(0, () -> run(actions.spinUpForDistance(3.25)))
                .build();

        // Path 2: (49,96) -> (16,84) control (81.6779913,79.9763863) | 142° -> 180°
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(49.0, 96.0),
                        new Pose(81.6779913, 79.9763863),
                        new Pose(16.0, 84.0)))
                .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 3: (16,84) -> (49,96) | 180° -> 142°
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(16.0, 84.0),
                        new Pose(49.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                // Re-spin to 39 in if needed
                //.addParametricCallback(0, () -> run(actions.spinUpForDistance(3.25)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 4: (49,96) -> (17,60) control (76.57311669,53.3884786) | 142° -> 180°
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(49.0, 96.0),
                        new Pose(76.57311669, 53.3884786),
                        new Pose(17.0, 65)))
                .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 5: (17,60) -> (14.0384047,72.10635156) control (73,75) | 180° -> 90°

        // Path 6: (14.0384,72.1063) -> (49,96) | 90° -> 142°
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(17, 65),
                        new Pose(49.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                // Re-spin to 39 in if needed
                //.addParametricCallback(0, () -> run(actions.spinUpForDistance(3.25)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 7: (49,96) -> (14.88921713,35.9468424) control (92.9512555,31.0546528) | 142° -> 180°
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(49.0, 96.0),
                        new Pose(92.9512555, 31.0546528),
                        new Pose(14.88921713, 35.9468424)))
                .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 8: (14.8892,35.9468) -> (48.7090103,95.9290989) | 180° -> 142°
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(14.88921713, 35.9468424),
                        new Pose(48.7090103, 95.9290989)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                // Re-spin to 39 in if needed
                .addParametricCallback(0, () -> run(actions.spinUpForDistance(3.25)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 9: (48.7090,95.9291) -> (10.42245195,10.2097488) control (14.25110782,54.664897) | 142° -> 270°
        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(48.7090103, 95.9290989),
                        new Pose(14.25110782, 54.664897),
                        new Pose(10.42245195, 10.2097488)))
                .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(270))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 10: (10.42245,10.20975) -> (48.9217344,95.9290898) | 270° -> 142°
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.42245195, 10.2097488),
                        new Pose(48.9217344, 95.9290898)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(142))
                // Re-spin to 39 in if needed
                .addParametricCallback(0, () -> run(actions.spinUpForDistance(3.25)))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Shoot after path 1 (39 in)
        PathChainTask path1Task = new PathChainTask(path1, 2)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(
                                new SleepAction(0.5),
                        actions.launch3()                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path1Task);

        addPath(path2, 0);

        // Shoot after path 3 (39 in)
        PathChainTask path3Task = new PathChainTask(path3, 1.5)
                .addWaitAction(
                        () -> true,
                        actions.launch3()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path3Task);

        addPath(path4, 1);
        addPath(path5, 0);

        // Shoot after path 6 (39 in)
        PathChainTask path6Task = new PathChainTask(path6, 1.5)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(
                                actions.threeBallabsoluteclose(39, 96.0)
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path6Task);

        addPath(path7, 0);

        // Shoot after path 8 (39 in)
        PathChainTask path8Task = new PathChainTask(path8, 1.5)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(
                                actions.threeBallabsoluteclose(39, 96.0)
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path8Task);

        addPath(path9, 0);

        // Shoot after path 10 (39 in)
        PathChainTask path10Task = new PathChainTask(path10, 1.5)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(
                                actions.threeBallabsoluteclose(39, 96.0)
                        )
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path10Task);
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
