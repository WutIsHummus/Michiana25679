package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.data.AngleUtils;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "inputtest")
public class inputtest extends PathChainAutoOpMode {

    private static final double FIELD_TICK_INCHES = 1.1811;
    private static final double MAX_X_INCHES = 22.0;
    private static final double MAX_Y_INCHES = 14.0;

    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private Timer opModeTimer;

    private final Pose startPose = new Pose(7, 55, Math.toRadians(0));
    private final Pose preloadPose = new Pose(42, 70, Math.toRadians(0));
    private final Pose intake = new Pose(11, 35, Math.toRadians(0));

    private PathChain scorePreload, depositgrab;
    private TurnTask firstGrabTurn;

    private double inputX = 0;  // inches offset from 48 (forward/back)
    private double inputY = 0;  // inches offset from 72 (side-to-side)
    private boolean firstTargetSet = false;
    private double targetAngleRadians = 0;
    private long lastInputTime = 0;

    @Override
    protected void buildPathChains() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0, () -> run(motorActions.safeServos()))
                .setZeroPowerAccelerationMultiplier(3)
                .build();
        depositgrab = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(preloadPose),
                        new Point(intake)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), intake.getHeading())
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0, () -> run(motorActions.safeServos()))
                .setZeroPowerAccelerationMultiplier(3)
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        addPath(scorePreload, 0);

        double deltaX = inputX;
        double deltaY = inputY;
        double distance = Math.hypot(deltaX, deltaY);
        double extendoTicks = Math.min(distance * 32.25, 800);

        targetAngleRadians = -(Math.atan2(deltaY, deltaX));  // y over x for proper heading

        addTurnToDegrees(Math.toDegrees(targetAngleRadians), 0)
                .addWaitAction(1, new SequentialAction(
                        motorActions.sampleExtend(extendoTicks),
                        motorActions.extendo.waitUntilFinished(),
                        motorActions.extendo.set(Math.min((distance + 3) * 32.25, 800)),
                        motorActions.spin.eat(),
                        motorActions.spin.eatUntilStrict(org.firstinspires.ftc.teamcode.helpers.data.Enums.DetectedColor.RED, motorControl)
                ));

        addPath(depositgrab, 0);

    }

    @Override
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected boolean isTurning() {
        if (!isActivelyTurningInternalFlag) return false;

        double headingError = AngleUtils.shortestAngleDifference(targetHeadingRadians, follower.getPose().getHeading());
        if (Math.abs(headingError) < HEADING_TOLERANCE || !follower.isBusy()) {
            isActivelyTurningInternalFlag = false;
            follower.breakFollowing();
            return false;
        }
        return true;
    }

    @Override
    protected double getCurrentTValue() {
        return follower.getCurrentTValue();
    }

    @Override
    protected void startPath(PathChainTask task) {
        isActivelyTurningInternalFlag = false;
        follower.followPath((PathChain) task.pathChain, true);
    }

    @Override
    public void init() {
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        buildPathChains();
    }

    @Override
    public void init_loop() {
        double deltaX = inputX;
        double deltaY = inputY;
        double distance = Math.hypot(deltaX, deltaY);
        double extendoTicks = Math.min(distance * 32.25, 800);
        long currentTime = System.currentTimeMillis();
        if (lastInputTime == 0) lastInputTime = currentTime;

        boolean inputAllowed = currentTime - lastInputTime > 100;

        if (inputAllowed) {
            if (gamepad1.dpad_up && inputX < MAX_X_INCHES) { inputX += FIELD_TICK_INCHES; lastInputTime = currentTime; }
            if (gamepad1.dpad_down && inputX > 0) { inputX -= FIELD_TICK_INCHES; lastInputTime = currentTime; }
            if (gamepad1.dpad_right && inputY < MAX_Y_INCHES) { inputY += FIELD_TICK_INCHES; lastInputTime = currentTime; }
            if (gamepad1.dpad_left && inputY > -MAX_Y_INCHES) { inputY -= FIELD_TICK_INCHES; lastInputTime = currentTime; }

            if (gamepad1.y && inputX < MAX_X_INCHES) { inputX += FIELD_TICK_INCHES; lastInputTime = currentTime; }
            if (gamepad1.a && inputX > 0) { inputX -= FIELD_TICK_INCHES; lastInputTime = currentTime; }
            if (gamepad1.b && inputY < MAX_Y_INCHES) { inputY += FIELD_TICK_INCHES; lastInputTime = currentTime; }
            if (gamepad1.x && inputY > -MAX_Y_INCHES) { inputY -= FIELD_TICK_INCHES; lastInputTime = currentTime; }

            if (gamepad1.right_bumper && !firstTargetSet) {
                lastInputTime = currentTime;
                buildTaskList();
                firstTargetSet = true;
            }
        }

        telemetry.addData("Target Mode", firstTargetSet ? "Ready" : "Target 1");
        telemetry.addData("X Offset (ticks)", (int)(inputX / FIELD_TICK_INCHES));
        telemetry.addData("Y Offset (ticks)", (int)(inputY / FIELD_TICK_INCHES));
        telemetry.addData("Degrees", Math.toDegrees(Math.atan2(inputY, inputX)));  // fixed here too
        telemetry.addData("Distance (in)", distance);
        telemetry.addData("Extendo Target Ticks", (int)(extendoTicks));
        telemetry.update();
    }

    @Override
    protected void startTurn(TurnTask task) {
        task.initiated = true;
        Pose currentPose = follower.getPose();
        double currentHeading = currentPose.getHeading();

        targetHeadingRadians = task.useDegrees ? Math.toRadians(task.angle) : task.angle;
        targetHeadingRadians = task.isRelative
                ? currentHeading + (task.isLeft ? targetHeadingRadians : -targetHeadingRadians)
                : targetHeadingRadians;
        targetHeadingRadians = AngleUtils.normalizeRadians(targetHeadingRadians);

        PathChain turnPath = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(currentPose)))
                .setConstantHeadingInterpolation(targetHeadingRadians)
                .build();

        follower.followPath(turnPath, true);
        isActivelyTurningInternalFlag = true;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
        run(motorActions.outtakeSpecimen());
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        runTasks();
        motorControl.update();

        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Turning", isTurning());
        telemetry.addData("PathActive", isPathActive());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Angle", Math.toDegrees(targetAngleRadians));
        telemetry.update();
    }
}
