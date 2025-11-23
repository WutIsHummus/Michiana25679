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

@Autonomous(name = "Test18ball")
public class Test18ball extends PathChainAutoOpMode {

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
        hood1       = hardwareMap.get(Servo.class, "hood 1");
        turret1     = hardwareMap.get(Servo.class, "turret1");
        turret2     = hardwareMap.get(Servo.class, "turret2");
        reargate    = hardwareMap.get(Servo.class, "reargate");
        launchgate  = hardwareMap.get(Servo.class, "launchgate");

        // Motor setup
        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        hood1.setPosition(0.5);      // default hood position
        turret1.setPosition(0.275);  // requested turret position
        turret2.setPosition(0.275);

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate, turret1, turret2, hood1
        );

        // Start Pose: rounded from (23.8227, 125.0694), heading 143° at start of Path1
        follower.setStartingPose(new Pose(27, 131, Math.toRadians(323)));

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
        run(actions.holdShooterAtRPMclose(1300, 30));
        runTasks();

        // Shooter debug
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
        // All poses rounded to whole numbers from your Paths class

        // Path 1: Start (24,125) -> (62.91) | 143° -> 230°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(27, 131),
                        new Pose(62, 91)))
                .setLinearHeadingInterpolation(Math.toRadians(323), Math.toRadians(230))
                .build();

        // Path 2: (62.91) -> (18,84) via (47,82) | 230° -> 170° (reversed)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(62, 91),
                        new Pose(47, 82),
                        new Pose(26, 84)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(170))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 3: (18,84) -> (62.91) | 170° -> 230°
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(26, 84),
                        new Pose(62, 91)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                // TODO: setZeroPowerAccelerationMultiplier removed in PedroPathing 2.0
                // .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 4: (62.91) -> (16,35) via (69,33) | 230° -> 180°
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(58, 85),
                        new Pose(69, 33),
                        new Pose(26, 35)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 5: (16,35) -> (62.91) via (63,44) | 180° -> 230°
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(26, 35),
                        new Pose(63, 44),
                        new Pose(62, 91)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                // TODO: setZeroPowerAccelerationMultiplier removed in PedroPathing 2.0
                // .setZeroPowerAccelerationMultiplier(8)

                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 6: (62.91) -> (17,62) via (46,57) | 230° -> 180°
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(62, 91),
                        new Pose(46, 57),
                        new Pose(22, 64)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 7: (17,62) -> (62.91) | 180° -> 230°
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22, 64),
                        new Pose(62, 91)))
                .setConstantHeadingInterpolation(Math.toRadians(230))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 8: (62.91) -> (11,10) via (12,48) | 230° -> 270°
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(62, 91),
                        new Pose(12, 48),
                        new Pose(13, 17)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(270))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 9: (11,10) -> (62.91) | 270° -> 230°
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(13, 10),
                        new Pose(62, 91)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(230))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 10: (62.91) -> (17,9) via (71,37) | 230° -> 180°
        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(62, 91),
                        new Pose(71, 37),
                        new Pose(17, 7)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 11: (17,9) -> (62.91) | 180° -> 230°
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(17, 9),
                        new Pose(62, 91)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Shoot after path 1 (preloads)
        PathChainTask p1 = new PathChainTask(path1, 1.5)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(actions.launch3())
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(p1);

        // 2–3: intake on 2, shoot after 3
        addPath(path2, 0);
        PathChainTask p3 = new PathChainTask(path3, 1.5)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(actions.launch3())
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(p3);

        // 4–5 pair
        addPath(path4, 0);
        PathChainTask p5 = new PathChainTask(path5, 1.5)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(actions.launch3())
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(p5);

        // 6–7 pair
        addPath(path6, 0);
        PathChainTask p7 = new PathChainTask(path7, 1.5)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(actions.launch3())
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(p7);

        // 8–9 pair
        addPath(path8, 0);
        PathChainTask p9 = new PathChainTask(path9, 1.5)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(actions.launch3())
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(p9);

        // 10–11 pair
        addPath(path10, 0);
        PathChainTask p11 = new PathChainTask(path11, 1.5)
                .addWaitAction(
                        () -> true,
                        new SequentialAction(actions.launch3())
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(p11);
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
            org.firstinspires.ftc.teamcode.opmodes.PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }

    @Override
    protected void startTurn(TurnTask task) {
        // Not used
    }
}
