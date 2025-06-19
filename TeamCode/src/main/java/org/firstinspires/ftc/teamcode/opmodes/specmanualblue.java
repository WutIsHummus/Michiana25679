package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.helpers.data.AngleUtils;
import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Specmanual blue")
public class specmanualblue extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private Timer opModeTimer;

    // -------- Poses --------
    private final Pose startPose    = new Pose(9,  67,   Math.toRadians(0));
    private final Pose preloadPose  = new Pose(41, 72.5, Math.toRadians(0));
    private final Pose preloadCopy  = new Pose(27, 72.5, Math.toRadians(0));
    private final Pose scorePose    = new Pose(41, 64,   Math.toRadians(0));
    private final Pose scorePose1   = new Pose(41, 67,   Math.toRadians(0));
    private final Pose scorePose2   = new Pose(41, 66,   Math.toRadians(0));
    private final Pose scorePose3   = new Pose(41, 65,   Math.toRadians(0));
    private final Pose scorePose4   = new Pose(41, 64,   Math.toRadians(0));
    private final Pose prescore     = new Pose(28, 64,   Math.toRadians(25));
    private final Pose thirdgrab    = new Pose(23, 22,   Math.toRadians(325));
    private final Pose pickup1Pose  = new Pose(20, 25,   Math.toRadians(0));
    private final Pose prepickup    = new Pose(20, 40,   Math.toRadians(25));
    private final Pose intake       = new Pose(11, 40,   Math.toRadians(0));
    private final Pose intakeDown       = new Pose(11, 24,   Math.toRadians(0));
    private final Pose parkPose     = new Pose(11, 22,   Math.toRadians(90));

    // -------- Manual “vision” inputs --------
    private static final double MAX_MANUAL_ANGLE_DEG   = 30;
    private static final double MAX_MANUAL_DISTANCE_IN = 48;
    private double manualAngle1    = 0, manualDistance1 = 0;
    private double manualAngle2    = 0, manualDistance2 = 0;
    // For edge-detecting D-pad presses:
    private boolean dpadLeftPrev  = false;
    private boolean dpadRightPrev = false;
    private boolean dpadUpPrev    = false;
    private boolean dpadDownPrev  = false;

    // How much each press moves you:
    private static final double ANGLE_STEP = 1.0;    // 1° per press
    private static final double DIST_STEP  = 1.18;    // 1 in per press

    // Clamp helper:
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private boolean aPrev = false, bPrev = false;
    private int currentManual = 0;
    // true = locked in, false = adjustable
    private boolean vision1done = false, vision2done = false;

    // -------- Internal state --------
    private boolean spitDone1 = false, spitDone2 = false;
    private boolean thirdspit = false, thirdgrabbed = false;

    private double manualHoriz1 = 0, manualHoriz2 = 0;
    // maximum horizontal offset
    private static final double MAX_MANUAL_HORIZ_IN = 48;
    // how much each D-pad ◀/▶ press moves you
    private static final double HORIZ_STEP = 1.0;

    private TurnTask visionTurn1, visionTurn2;
    private double latestVisionAngle = 0, lastDistance = 0;

    private final AtomicBoolean specimenProcessingComplete = new AtomicBoolean(false);

    // -------- PathChains --------
    private PathChain scorePreload, parkChain;
    private PathChain intake1, intake2, intake3, intake4, intake5;
    private PathChain score1, score2, score3, score4, score5;
    private PathChain vision1deposit, vision2intake, vision2deposit, thirdgrabdrive;

    @Override
    public void init() {
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        follower     = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        // Prompt user to set manual inputs
        telemetry.addLine("← stick = angle, → stick = distance");
        telemetry.addLine("Press Ⓐ → set DEPOSIT 1");
        telemetry.addLine("Press Ⓑ → set DEPOSIT 2");
        telemetry.update();

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void init_loop() {
        // —— select deposit on A/B rising edge ——
        if (gamepad1.a && !aPrev) currentManual = (currentManual == 1 ? 0 : 1);
        aPrev = gamepad1.a;
        if (gamepad1.b && !bPrev) currentManual = (currentManual == 2 ? 0 : 2);
        bPrev = gamepad1.b;

        // —— adjust selected deposit’s horiz & forward dist with D-pad ——
        if (currentManual == 1) {
            if (gamepad1.dpad_left  && !dpadLeftPrev)  manualHoriz1    = clamp(manualHoriz1 - HORIZ_STEP,    -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_right && !dpadRightPrev) manualHoriz1    = clamp(manualHoriz1 + HORIZ_STEP,    -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_up    && !dpadUpPrev)    manualDistance1 = clamp(manualDistance1 + DIST_STEP, 0,                  MAX_MANUAL_DISTANCE_IN);
            if (gamepad1.dpad_down  && !dpadDownPrev)  manualDistance1 = clamp(manualDistance1 - DIST_STEP, 0,                  MAX_MANUAL_DISTANCE_IN);

            double centerDist1 = manualDistance1 + 7;
            // avoid division-by-zero or negative:
            centerDist1 = Math.max(centerDist1, 0.1);

            // compute the angle:
            manualAngle1 = Math.toDegrees(Math.atan2(manualHoriz1, centerDist1));
        }
        else if (currentManual == 2) {
            if (gamepad1.dpad_left  && !dpadLeftPrev)  manualHoriz2    = clamp(manualHoriz2 - HORIZ_STEP,    -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_right && !dpadRightPrev) manualHoriz2    = clamp(manualHoriz2 + HORIZ_STEP,    -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_up    && !dpadUpPrev)    manualDistance2 = clamp(manualDistance2 + DIST_STEP, 0,                  MAX_MANUAL_DISTANCE_IN);
            if (gamepad1.dpad_down  && !dpadDownPrev)  manualDistance2 = clamp(manualDistance2 - DIST_STEP, 0,                  MAX_MANUAL_DISTANCE_IN);

            double centerDist2 = manualDistance2 + 7;
            // avoid division-by-zero or negative:
            centerDist2 = Math.max(centerDist2, 0.1);

            // compute the angle:
            manualAngle2 = Math.toDegrees(Math.atan2(manualHoriz2, centerDist2));
        }

        // —— update edge-detect flags ——
        dpadLeftPrev  = gamepad1.dpad_left;
        dpadRightPrev = gamepad1.dpad_right;
        dpadUpPrev    = gamepad1.dpad_up;
        dpadDownPrev  = gamepad1.dpad_down;

        // —— telemetry ——
        telemetry.addLine("Press A/B to pick deposit 1 or 2");
        telemetry.addLine("DPAD ◀▶ = horizontal, ▲▼ = forward dist");
        telemetry.addData("Sel",   currentManual == 0 ? "none" : "dep " + currentManual);
        telemetry.addData("1 h,d,a",String.format("(%.1f, %.1f, %.1f°)", manualHoriz1, manualDistance1, manualAngle1) );
        telemetry.addData("2 h,d,a", String.format("(%.1f, %.1f, %.1f°)", manualHoriz2, manualDistance2, manualAngle2));
        telemetry.update();
    }




    @Override
    public void start() {
        super.start();
        // Lock in first manual values before any turns
        latestVisionAngle = -manualAngle1;
        lastDistance      = manualDistance1;

        opModeTimer.resetTimer();
        currentTaskIndex = 0;
        taskPhase        = 0;
        pathTimer.resetTimer();
        run(motorActions.outtakespecvision());
    }

    @Override
    protected void buildPathChains() {
        // 1) Preload score
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .addParametricCallback(0,   () -> run(motorActions.outtakespecvision()))
                .addParametricCallback(0, ()->run(motorActions.sweeper.retracted()))
                .addParametricCallback(0,   () -> run(motorActions.safeServos()))
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0.2, () -> run(motorActions.specimenExtend(0)))
                .build();

        // 2) “Vision” deposit → intake
        vision1deposit = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPose), new Point(prepickup), new Point(intake)))
                .addParametricCallback(0, ()->run(motorActions.extendo.set(0)))
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.spitSamplettele(),
                        motorActions.spin.waitUntilEmpty(motorControl),
                        motorActions.lift.transfer())))
                .addParametricCallback(0.7, () -> run(motorActions.spin.poop()))
                .addParametricCallback(0.8,()-> run(motorActions.lift.transfer()))
                .addParametricCallback(1,()-> run(motorActions.claw.close()))
                .setConstantHeadingInterpolation(intake.getHeading())
                .build();

        // 3) “Vision” intake → preload
        vision2intake = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(intake), new Point(preloadCopy), new Point(preloadPose)))
                .addParametricCallback(0,     () -> run(new SequentialAction(motorActions.spin.stop(), motorActions.sampleExtend(0))))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .build();

        // 4) “Vision” deposit at pickup1
        vision2deposit = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(preloadCopy), new Point(pickup1Pose)))
                .addParametricCallback(0, () -> run(motorActions.extendo.set(0)))
                .addParametricCallback(0,    () -> run(motorActions.spin.stop()))
                .addParametricCallback(0,    () -> run(motorActions.spitSamplettele()))
                .addParametricCallback(0.7, () -> run(motorActions.spin.poop()))
                .addParametricCallback(1,    () -> run(motorActions.extendo.extended()))
                .setZeroPowerAccelerationMultiplier(3)
                .setConstantHeadingInterpolation(Math.toRadians(pickup1Pose.getHeading()))
                .build();

        // 5) Third grab drive
        thirdgrabdrive = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(thirdgrab)))
                .addParametricCallback(0,()->run(motorActions.extendo.set(600)))
                .addParametricCallback(0, () -> run(new ParallelAction(
                        motorActions.grabUntilSpecimen())))
                .setConstantHeadingInterpolation(thirdgrab.getHeading())
                .build();

        // 6) Intake1
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdgrab), new Point(intakeDown)))
                .setConstantHeadingInterpolation(intake.getHeading())
                .addParametricCallback(0.9, () -> motorControl.spin.setPower(-1))
                .addParametricCallback(0,   () -> run(motorActions.spitSample()))
                .addParametricCallback(0,   () -> run(motorActions.intakeSpecimen()))
                .build();

        // 7) Score1
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(intake), new Point(prescore), new Point(scorePose1)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0, () -> motorControl.spin.setPower(0))
                .build();

        // 8) Intake2
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(prepickup), new Point(intake)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setReversed(true)
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0,   () -> run(motorActions.specgone()))
                .build();

        // 9) Score2
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(intake), new Point(prescore), new Point(scorePose2)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        // 10) Intake3
        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(prepickup), new Point(intake)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setReversed(true)
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0,   () -> run(motorActions.specgone()))
                .build();

        // 11) Score3
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(intake), new Point(prescore), new Point(scorePose3)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        // 12) Intake4
        intake4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(prepickup), new Point(intake)))
                .setConstantHeadingInterpolation(scorePose1.getHeading())
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setZeroPowerAccelerationMultiplier(7)
                .setReversed(true)
                .addParametricCallback(0,   () -> run(motorActions.specgone()))
                .build();

        // 13) Score4
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(intake), new Point(prescore), new Point(scorePose4)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        // 14) Intake5
        intake5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(prepickup), new Point(intake)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setZeroPowerAccelerationMultiplier(7)
                .setReversed(true)
                .addParametricCallback(0,   () -> run(motorActions.specgone()))
                .build();

        // 15) Score5
        score5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(intake), new Point(prescore), new Point(scorePose4)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0,   () -> run(motorActions.outtakeSpecimen()))
                .build();

        // 16) Park
        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose4), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // 1) Preload
        addPath(scorePreload, 0)
                .addWaitAction(0, new ParallelAction(
                        motorActions.claw.open(),
                        motorActions.outArm.pretransfer(),
                        motorActions.linkage.retracted()
                ));

        // 2) First manual turn
        visionTurn1 = addRelativeTurnDegrees(0, true, 0)
                .setWaitCondition(()->!motorControl.isEmpty())
                .setMaxWaitTime(2);

        // 3) First vision deposit
        addPath(vision1deposit, 0.1)
                .addWaitAction(0,   motorActions.outtakespecvision());

        // 4) Intake after deposit
        addPath(vision2intake, 0)
                .addWaitAction(0, new ParallelAction(
                        motorActions.claw.open(),
                        motorActions.outArm.pretransfer(),
                        motorActions.linkage.retracted()
                ));

        // 5) Second manual turn
        visionTurn2 = addTurnToDegrees(0, 0)
                .setWaitCondition(()->!motorControl.isEmpty())
                .setMaxWaitTime(2);

        // 6) Second vision deposit
        addPath(vision2deposit, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.inArm.specimenGrab(),
                        motorActions.inPivot.specimenGrab(),
                        new SleepAction(0.05),
                        motorActions.extendo.set(640),
                        motorActions.spin.eat(),
                        motorActions.extendo.waitUntilFinished(640),
                        motorActions.spitSampleautofast(),
                        telemetryPacket -> {
                            spitDone1 = true; return false;
                        }
                ))
                .setMaxWaitTime(1.5)
                .setWaitCondition(() -> spitDone1);

        // 7) Additional scoring
        addTurnToDegrees(-21, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.spin.waitUntilEmpty(motorControl),
                        motorActions.inArm.specimenGrab(),
                        motorActions.inPivot.specimenGrab(),
                        new SleepAction(0.05),
                        motorActions.extendo.set(640),
                        motorActions.spin.eat(),
                        motorActions.extendo.waitUntilFinished(),
                        new SleepAction(0.05),
                        motorActions.spitSampleautofast(),
                        telemetryPacket -> {
                            spitDone2 = true; return false;
                        },
                        motorActions.extendo.waitUntilFinished(),
                        motorActions.inPivot.specimenExtended()
                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> spitDone2);

        // 8) Third grab & spit
        tasks.add(new PathChainTask(thirdgrabdrive, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.set(600),
                        motorActions.grabUntilSpecimen(),
                        telemetryPacket -> { thirdgrabbed = true; return false; },
                        motorActions.spitSample()
                ))
                .setMaxWaitTime(1)
                .setWaitCondition(() -> thirdgrabbed)
        );

        // 9) Intake1 follow-up
        tasks.add(new PathChainTask(intake1, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.extendo.waitUntilFinished(),
                        telemetryPacket -> { thirdspit = true; return false; }
                ))
                .setWaitCondition(() -> thirdspit)
        );

        // 10) Score1…Score5 & Intake2…Intake5
        tasks.add(new PathChainTask(score1, 0));
        tasks.add(new PathChainTask(intake2, 0));
        tasks.add(new PathChainTask(score2, 0));
        tasks.add(new PathChainTask(intake3, 0));
        tasks.add(new PathChainTask(score3, 0));
        tasks.add(new PathChainTask(intake4, 0));
        tasks.add(new PathChainTask(score4, 0));
        tasks.add(new PathChainTask(intake5, 0));
        tasks.add(new PathChainTask(score5, 0));

        // 11) Park
        tasks.add(new PathChainTask(parkChain, 0));
    }

    @Override
    protected boolean isPathActive() {
        BaseTask current = tasks.get(currentTaskIndex);
        if (current instanceof TurnTask) {
            return isTurning();
        }
        PathChainTask p = (PathChainTask) current;
        return p.initiated && follower.isBusy();
    }

    @Override
    protected boolean isTurning() {
        if (!isActivelyTurningInternalFlag) {
            return false;
        }
        double currentHeading = follower.getPose().getHeading();
        double headingError   = AngleUtils.shortestAngleDifference(targetHeadingRadians, currentHeading);
        if (Math.abs(headingError) < HEADING_TOLERANCE) {
            isActivelyTurningInternalFlag = false;
            return false;
        }
        if (!follower.isBusy()) {
            isActivelyTurningInternalFlag = false;
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
    protected void startTurn(TurnTask task) {
        // Before second turn, swap in deposit 2 readings
        if (task == visionTurn2) {
            latestVisionAngle = -manualAngle2;
            lastDistance      = manualDistance2;
        }

        if ((task == visionTurn1 || task == visionTurn2) && lastDistance > 0) {
            task.addWaitAction(0, new ParallelAction(
                    new SequentialAction(
                            motorActions.sampleExtend(Math.min(lastDistance * 32.25, 800)),
                            motorActions.extendo.waitUntilFinished(Math.min(lastDistance * 32.25, 800), 300),
                            motorActions.spin.eat(),
                            motorActions.spin.eatUntilStrict(Enums.DetectedColor.BLUE, motorControl),
                            telemetryPacket -> {
                                lastDistance = 0;
                                return false;
                            },
                            motorActions.spitSamplettele())
            ))
                    .addWaitAction(()-> motorControl.getDetectedColor() == Enums.DetectedColor.BLUE, motorActions.extendo.set(0))
                    .addWaitAction(()-> motorControl.getDetectedColor() == Enums.DetectedColor.RED, motorActions.spin.poop())
            ;
            task.angle      = Math.abs(latestVisionAngle);
            task.isLeft     = (latestVisionAngle > 0);
            task.useDegrees = true;
            task.isRelative = true;
        }

        // Standard turn setup
        task.initiated = true;
        Pose pose      = follower.getPose();
        double curH    = pose.getHeading();

        if (task.isRelative) {
            double delta = task.useDegrees
                    ? Math.toRadians(task.angle)
                    : task.angle;
            targetHeadingRadians = curH + (task.isLeft ? delta : -delta);
        } else {
            targetHeadingRadians = task.useDegrees
                    ? Math.toRadians(task.angle)
                    : task.angle;
        }
        targetHeadingRadians = AngleUtils.normalizeRadians(targetHeadingRadians);

        PathChain turnPath = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(new Pose(pose.getX(), pose.getY(), curH))))
                .setPathEndHeadingConstraint(0.001)
                .setConstantHeadingInterpolation(targetHeadingRadians)
                .setZeroPowerAccelerationMultiplier(10)
                .build();
        follower.followPath(turnPath, true);
        isActivelyTurningInternalFlag = true;
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        runTasks();
        motorControl.update();

        telemetry.addData("Task Index",    currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase",         taskPhase == 0 ? "DRIVE" : "WAIT");
        telemetry.addData("T Value",       follower.getCurrentTValue());
        telemetry.addData("Wait Timer",    pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Turning",       isTurning());
        telemetry.addData("PathActive",    isPathActive());
        telemetry.addData("Busy",          follower.isBusy());
        telemetry.addData("liftCurrent",   motorControl.lift.motor.getCurrentPosition());
        telemetry.addData("liftTarget",    motorControl.lift.getTargetPosition());
        telemetry.addData("isEmpty",       motorControl.isEmpty());
        telemetry.addData("extendoReset",  motorControl.extendo.resetting);
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.addData("Angle",         latestVisionAngle);
        telemetry.addData("Distance",      lastDistance);
        telemetry.addData("spitDone1",      spitDone1);
        telemetry.addData("spitDone2",      spitDone2);
        telemetry.update();
    }
}
