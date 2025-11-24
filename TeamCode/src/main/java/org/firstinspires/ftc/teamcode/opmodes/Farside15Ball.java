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

@Autonomous(name = "Farside15shootfarther")
public class Farside15Ball extends PathChainAutoOpMode {

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

        // Set motor directions
        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        hood1.setPosition(0.5);   // default hood position (close/auto)
        turret1.setPosition(0.51); // center
        turret2.setPosition(0.51);

        actions = new RobotActions(intakefront, intakeback, shootr, shootl, launchgate, reargate, turret1, turret2, hood1);

        // === START POSE (from canvas) ===
        // Start Point: X: 57.004431, Y: 9.146233; start heading ~90°
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
        run(actions.holdShooterAtRPMclose(1200, 30));
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
        // Path 1: Start -> (49, 96) | heading 90° -> 134°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(57.004431, 9.146233),
                        new Pose(49.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(134))
                .build();

        // Path 2: (49,96) -> (17.228951, 83.592319) via (81.677991, 79.976366) | 133° -> 180°
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(49.0, 96.0),
                        new Pose(81.677991, 79.976366),
                        new Pose(21, 83.592319)))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 3: (17.228951,83.592319) -> (49,96) | 180° -> 134°
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(21, 83.592319),
                        new Pose(49.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 4: (49,96) -> (9.358936,55.515510) via (73.382570,55.090103) | 134° -> 180°
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(49.0, 96.0),
                        new Pose(73.382570, 55.090103),
                        new Pose(11, 55.515510)))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 5: (9.358936,55.515510) -> (16.378139,72.106352) via (73,75) | 180° -> 90°

        // Path 6: (16.378139,72.106352) -> (49,96) | 90° -> 133°
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(14, 55.515510),
                        new Pose(49.0, 96.0)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 7: (49,96) -> (9.997046,34.883309) via (92.951256,31.054653) | 134° -> 180°
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(49.0, 96.0),
                        new Pose(92.951256, 31.054653),
                        new Pose(9.997046, 34.883309)))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 8: (9.997046,34.883309) -> (48.709010,95.929099) | 180° -> 132°
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(9.997046, 34.883309),
                        new Pose(48.709010, 95.929099)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 9: (48.709010,95.929099) -> (10.422452,10.209749) via (19.143279,53.388479) | 134° -> 270°
        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(48.709010, 95.929099),
                        new Pose(19.143279, 53.388479),
                        new Pose(10.422452, 10.209749)))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(270))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // Path 10: (10.422452,10.209749) -> (48.921713,95.929099) | 270° -> 134°
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.422452, 10.209749),
                        new Pose(48.921713, 95.929099)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(134))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(38.2865858, 105.075332),

                        new Pose(52, 120)))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(134))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Shoot after path 1
        PathChainTask path1Task = new PathChainTask(path1, 1.5)
                .addWaitAction(
                        () -> true,
                        actions.launch3()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path1Task);

        addPath(path2, 0);

        // Shoot after path 3
        PathChainTask path3Task = new PathChainTask(path3, 1.5)
                .addWaitAction(
                        () -> true,
                        actions.launch3()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path3Task);

        addPath(path4, 1);
        //addPath(path5, 0);

        // Shoot after path 6
        PathChainTask path6Task = new PathChainTask(path6, 1.5)
                .addWaitAction(
                        () -> true,
                        actions.launch3()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path6Task);

        addPath(path7, 0);

        // Shoot after path 8
        PathChainTask path8Task = new PathChainTask(path8, 1.5)
                .addWaitAction(
                        () -> true,
                        actions.launch3()
                )
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true);
        tasks.add(path8Task);

        addPath(path9, 0);

        // Shoot after path 10
        PathChainTask path10Task = new PathChainTask(path10, 1.5)
                .addWaitAction(
                        () -> true,
                        actions.launch3()
                )
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
