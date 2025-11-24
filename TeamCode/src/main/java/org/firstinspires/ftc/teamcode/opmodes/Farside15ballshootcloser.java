package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "Farblueshootcloser")
public class Farside15ballshootcloser extends PathChainAutoOpMode {

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
        hood1.setPosition(0.54);   // default hood position (close/auto)
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
        run(actions.holdShooterAtRPMclose(1350, 30));
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
        // Path 1  (has control pt)
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(57.0044313, 9.14623338),
                        new Pose(63.3855424, 36.3540521),
                        new Pose(38.2865858, 105.288035)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(134))
                .build();

        // Path 2  (has control pt)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(38.2865858, 105.288035),
                        new Pose(81.6779913, 79.9763663),
                        new Pose(23.1846831, 84.2304238)))
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path 3  (line)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(23.1846831, 84.2304238),
                        new Pose(38.4992614, 105.075332)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 4  (has control pt)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(38.4992614, 105.075332),
                        new Pose(89.9734121, 49.34711964),
                        new Pose(9.51763958, 57.0044313)))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path 5  (from UI “Path 6”; has control pt)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(9.51763958, 57.0044313),
                        new Pose(45.73116691, 63.8109305),
                        new Pose(38.2865858, 105.288035)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 6  (from UI “Path 7”; has control pt)
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(38.2865858, 105.288035),
                        new Pose(81.6779913, 34.2451994),
                        new Pose(9.99704576, 34.8833087)))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path 7  (from UI “Path 8”; has control pt)
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(9.99704576, 34.8833087),
                        new Pose(58.4933535, 55.8490158),
                        new Pose(38.4992614, 105.075332)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();

        // Path 8  (from UI “Path 9”; has control pt)
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(38.4992614, 105.075332),
                        new Pose(19.14327917, 53.3884786),
                        new Pose(10.42245195, 10.2097488)))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(270))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // Path 9  (from UI “Path 9” second; has control pt)
        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(10.42245195, 10.2097488),
                        new Pose(49.34711964, 47.2200886),
                        new Pose(38.2865858, 105.075332)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(134))
                .addParametricCallback(0.5, () -> run(actions.stopIntake()))
                .build();
        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(38.2865858, 105.075332),

                        new Pose(52, 120)))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(134))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Shoot after Path 1
        tasks.add(new PathChainTask(path1, 1.5)
                .addWaitAction(() -> true, actions.launch3())
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true));

        addPath(path2, 0);

        // Shoot after Path 3
        tasks.add(new PathChainTask(path3, 1.5)
                .addWaitAction(() -> true, actions.launch3())
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true));

        addPath(path4, 0);

        // Shoot after Path 5
        tasks.add(new PathChainTask(path5, 1.5)
                .addWaitAction(() -> true, actions.launch3())
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true));

        addPath(path6, 0);

        // Shoot after Path 7
        tasks.add(new PathChainTask(path7, 1.5)
                .addWaitAction(() -> true, actions.launch3())
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true));

        addPath(path8, 0);

        // Shoot after Path 9 (final)
        tasks.add(new PathChainTask(path9, 1.5)
                .addWaitAction(() -> true, actions.launch3())
                .setMaxWaitTime(6.0)
                .setWaitCondition(() -> true));
        addPath(path10, 0);
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
