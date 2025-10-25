package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
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


@Autonomous(name = "Samplemanualcomptest")
public class samplemanualcomptest extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private Timer opModeTimer;  // additional timer if desired

    // -------- Poses --------
    private final Pose startPose   = new Pose(9, 111, Math.toRadians(270));
    private final Pose scorePose   = new Pose(18, 130, Math.toRadians(340));

    private final Pose subscorepose   = new Pose(23, 131, Math.toRadians(340));
    private final Pose thirdgrab = new Pose(18.5, 129, Math.toRadians(25));

    // Park poses
    private final Pose parkPose        = new Pose(62, 101, Math.toRadians(270));

    private final Pose subintakepose        = new Pose(62, 97, Math.toRadians(270));
    private final Pose parkControlPose = new Pose(53, 120, Math.toRadians(270));

    // -------- Manual “vision” inputs --------
    private static final double MAX_MANUAL_ANGLE_DEG   = 30;
    private static final double MAX_MANUAL_DISTANCE_IN = 48;
    private double manualAngle1 = 0, manualDistance1 = 0;
    private double manualAngle2 = 0, manualDistance2 = 0;
    private double manualAngle3 = 0, manualDistance3 = 0;
    private boolean dpadLeftPrev  = false, dpadRightPrev = false;
    private boolean dpadUpPrev    = false, dpadDownPrev  = false;

    private static final double ANGLE_STEP = 1.0;    // 1° per press
    private static final double DIST_STEP  = 1.18;   // 1 in per press

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private boolean aPrev = false, bPrev = false, yPrev = false;
    private int currentManual = 0;

    private double manualHoriz1 = 0, manualHoriz2 = 0, manualHoriz3 = 0;
    private static final double MAX_MANUAL_HORIZ_IN = 48;
    private static final double HORIZ_STEP = 1.0;


    private boolean eatdone1, eatdone2, eatdone3, firstcycle, depodone1, depodone2 = false;

    // --- Vision turn related fields ---
    private TurnTask visionTurn1, visionTurn2, visionTurn3;

    private double latestVisionAngle = 0;
    private double lastDistance = 0;

    private boolean subscoreDone1 = false;
    private boolean subscoreDone2 = false;
    private boolean subscoreDone3 = false;


    private final AtomicBoolean specimenProcessingComplete = new AtomicBoolean(false);


    // -------- PathChains --------
    private PathChain scorePreload;
    private PathChain intake1, intake2, intake3;
    private PathChain score1, score2, score3;

    private PathChain subscore1, subscore2, subscore3;
    private PathChain parkChain1, parkChain2, parkChain3, thirdgrabpath;

    private PathChain parkToControl, controlToSubintake;

    /**
     * Collects a single set of Limelight samples and stores the results
     * into {@code latestVisionPose} (horizontal, forward) and
     * {@code latestVisionAngle}. Returns true if successful.
     */


    // -------- Override buildPathChains() --------
    @Override
    protected void buildPathChains() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0,   () -> run(motorActions.safeServos()))
                .addParametricCallback(0, () -> run(motorActions.outtakeSampleAuto()))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        thirdgrabpath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(thirdgrab)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdgrab.getHeading())
                .addParametricCallback(0.9, () -> run(motorActions.extendo.set(100)))
                .build();

        // buildPathChains(): split your park chain in two
        parkToControl = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.inArm.sampleExtended(),
                        motorActions.inPivot.sampleExtended()
                )))
                .build();



        parkChain2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(),parkPose.getHeading())
                .addParametricCallback(0.2, () -> run(motorActions.outtakeTransfer()))
                .addParametricCallback(0, () -> run(motorActions.extendo.retracted()))
                .addParametricCallback(0.5, () -> run(new ParallelAction(
                        motorActions.inArm.sampleExtended(),
                        motorActions.inPivot.sampleExtended()
                )))
                .build();


        parkChain3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(),parkPose.getHeading())
                .addParametricCallback(0.2, () -> run(motorActions.outtakeTransfer()))
                .addParametricCallback(0, () -> run(motorActions.extendo.retracted()))
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakeTransfer(),
                        motorActions.outtakeSampleAuto2()
                )))
                .build();



        subscore1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakeTransferAuto(),
                        new ParallelAction(
                                motorActions.outtakeSampleAuto2(),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        motorActions.extendo.set(150)
                                )
                        )
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
                .build();

        subscore2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakeTransferAuto(),
                        new ParallelAction(
                                motorActions.outtakeSampleAuto2(),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        motorActions.extendo.set(150)
                                )
                        )
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
                .build();

        subscore3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(parkPose), new Point(parkControlPose), new Point(subscorepose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakeTransferAuto(),
                        new ParallelAction(
                                motorActions.outtakeSampleAuto2(),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        motorActions.extendo.set(150)
                                )
                        )
                )))
                .addParametricCallback(1, () -> run(motorActions.claw.open()))
                .build();



    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        PathChainTask preloadTask = new PathChainTask(scorePreload, 0.05)
                .addWaitAction(0, motorActions.extendo.set(300))
                .addWaitAction(0, new SequentialAction(
                        motorActions.hang.down(),
                        new ParallelAction(
                                motorActions.resetToZero(),
                                motorActions.cycle()
                                ),


                        telemetryPacket -> {
                            firstcycle = true; return false;
                        }

                ))
                .setWaitCondition(() -> firstcycle);

        tasks.add(preloadTask);
        addTurnToDegrees(-2, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.spin.eat(),
                        motorActions.extendo.set(600),
                        new SleepAction(0.05),
                        motorActions.inArm.specimenGrab(),
                        motorActions.inPivot.specimenGrab(),
                        motorActions.spin.eat(),
                        motorActions.extendo.waitUntilFinished(),
                        telemetryPacket -> {
                            eatdone1 = true; return false;
                        },
                        motorActions.intakeTransfer(),
                        motorActions.extendo.waitUntilFinished()
                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> eatdone1)
        ;

        addTurnToDegrees(340, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeSampleAuto(),
                        telemetryPacket -> {
                            depodone1 = true; return false;
                        }

                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> depodone1)
        ;

        addPath(thirdgrabpath,0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.spin.eat(),
                        //new SleepAction(0.05),
                        motorActions.extendo.set(660),
                        motorActions.inArm.specimenGrab(),
                        motorActions.inPivot.specimenGrab(),
                        motorActions.extendo.waitUntilFinished(),
                        telemetryPacket -> {
                            eatdone2 = true; return false;
                        },
                        motorActions.intakeTransfer(),
                        motorActions.extendo.waitUntilFinished()
                ))
                .setMaxWaitTime(1)
                .setWaitCondition(() -> eatdone2);

        addTurnToDegrees(330, 0)
                .addWaitAction(0, new SequentialAction(
                        motorActions.outtakeSampleAuto(),
                        motorActions.lift.waitUntilFinished(),
                        telemetryPacket -> {
                            depodone2 = true; return false;
                        }

                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> depodone2)
        ;

        addPath(parkToControl,  0);


        visionTurn1 = addRelativeTurnDegrees(0, true, 0)
                .setWaitCondition(()->!motorControl.isEmpty())
                .setMaxWaitTime(2);

        addPath(subscore1, 0)
                .addWaitAction(0, motorActions.outArm.sampleScoreAuto())
                .addWaitAction(0.1, new SequentialAction(
                        motorActions.outtakeTransfer(),
                        telemetryPacket -> {
                            subscoreDone1 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(0.7)
                .setWaitCondition(() -> subscoreDone1);

// Manual deposit 2 setup
        addPath(parkChain2, 0);


        visionTurn2 = addRelativeTurnDegrees(0, true, 0)
                .setWaitCondition(()->!motorControl.isEmpty())
                .setMaxWaitTime(2);

        addPath(subscore2, 0)
                .addWaitAction(0, motorActions.outArm.sampleScoreAuto())
                .addWaitAction(0.1, new SequentialAction(
                        motorActions.outtakeTransfer(),
                        telemetryPacket -> {
                            subscoreDone2 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(0.7)
                .setWaitCondition(() -> subscoreDone2);

// Manual deposit 3 setup
        addPath(parkChain3, 0);

        visionTurn3 = addRelativeTurnDegrees(0, true, 0)
                .setWaitCondition(()->!motorControl.isEmpty())
                .setMaxWaitTime(2);

        addPath(subscore3, 0)
                .addWaitAction(0, motorActions.outArm.sampleScoreAuto())
                .addWaitAction(0.1, new SequentialAction(
                        motorActions.outtakeTransfer(),
                        telemetryPacket -> {
                            subscoreDone3 = true;
                            return false;
                        }
                ))
                .setMaxWaitTime(0.7)
                .setWaitCondition(() -> subscoreDone3);


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

        // This is just an example, replace with your actual follower's heading retrieval
        double currentHeading = follower.getPose().getHeading();

        // Calculate the shortest difference, handling wrap-around
        double headingError = AngleUtils.shortestAngleDifference(targetHeadingRadians ,currentHeading);

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

    // -------- Standard OpMode Lifecycle Methods --------
    @Override
    public void init() {
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);







        // Build the paths and tasks.
        buildPathChains();
        buildTaskList();
    }

    @Override
    public void init_loop() {
        // —— select deposit on A/B/Y rising edge ——
        if (gamepad1.a && !aPrev) currentManual = (currentManual == 1 ? 0 : 1);
        aPrev = gamepad1.a;
        if (gamepad1.b && !bPrev) currentManual = (currentManual == 2 ? 0 : 2);
        bPrev = gamepad1.b;
        if (gamepad1.y && !yPrev) currentManual = (currentManual == 3 ? 0 : 3);
        yPrev = gamepad1.y;

        // —— adjust selected deposit’s horiz & forward dist with D-pad ——
        if (currentManual == 1) {
            if (gamepad1.dpad_left  && !dpadLeftPrev)  manualHoriz1    = clamp(manualHoriz1 - HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_right && !dpadRightPrev) manualHoriz1    = clamp(manualHoriz1 + HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_up    && !dpadUpPrev)    manualDistance1 = clamp(manualDistance1 + DIST_STEP, 0, MAX_MANUAL_DISTANCE_IN);
            if (gamepad1.dpad_down  && !dpadDownPrev)  manualDistance1 = clamp(manualDistance1 - DIST_STEP, 0, MAX_MANUAL_DISTANCE_IN);

            double centerDist = Math.max(manualDistance1 + 7, 0.1);
            manualAngle1 = Math.toDegrees(Math.atan2(manualHoriz1, centerDist));
        } else if (currentManual == 2) {
            if (gamepad1.dpad_left  && !dpadLeftPrev)  manualHoriz2    = clamp(manualHoriz2 - HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_right && !dpadRightPrev) manualHoriz2    = clamp(manualHoriz2 + HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_up    && !dpadUpPrev)    manualDistance2 = clamp(manualDistance2 + DIST_STEP, 0, MAX_MANUAL_DISTANCE_IN);
            if (gamepad1.dpad_down  && !dpadDownPrev)  manualDistance2 = clamp(manualDistance2 - DIST_STEP, 0, MAX_MANUAL_DISTANCE_IN);

            double centerDist = Math.max(manualDistance2 + 7, 0.1);
            manualAngle2 = Math.toDegrees(Math.atan2(manualHoriz2, centerDist));
        } else if (currentManual == 3) {
            if (gamepad1.dpad_left  && !dpadLeftPrev)  manualHoriz3    = clamp(manualHoriz3 - HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_right && !dpadRightPrev) manualHoriz3    = clamp(manualHoriz3 + HORIZ_STEP, -MAX_MANUAL_HORIZ_IN, MAX_MANUAL_HORIZ_IN);
            if (gamepad1.dpad_up    && !dpadUpPrev)    manualDistance3 = clamp(manualDistance3 + DIST_STEP, 0, MAX_MANUAL_DISTANCE_IN);
            if (gamepad1.dpad_down  && !dpadDownPrev)  manualDistance3 = clamp(manualDistance3 - DIST_STEP, 0, MAX_MANUAL_DISTANCE_IN);

            double centerDist = Math.max(manualDistance3 + 7, 0.1);
            manualAngle3 = Math.toDegrees(Math.atan2(manualHoriz3, centerDist));
        }

        dpadLeftPrev  = gamepad1.dpad_left;
        dpadRightPrev = gamepad1.dpad_right;
        dpadUpPrev    = gamepad1.dpad_up;
        dpadDownPrev  = gamepad1.dpad_down;

        telemetry.addLine("Press A/B/Y to pick deposit 1/2/3");
        telemetry.addLine("DPAD \u25C0\u25B6 = horizontal, \u25B2\u25BC = forward dist");
        telemetry.addData("Sel",   currentManual == 0 ? "none" : "dep " + currentManual);
        telemetry.addData("1 h,d,a", String.format("(%.1f, %.1f, %.1f\u00B0)", manualHoriz1, manualDistance1, manualAngle1));
        telemetry.addData("2 h,d,a", String.format("(%.1f, %.1f, %.1f\u00B0)", manualHoriz2, manualDistance2, manualAngle2));
        telemetry.addData("3 h,d,a", String.format("(%.1f, %.1f, %.1f\u00B0)", manualHoriz3, manualDistance3, manualAngle3));
        telemetry.update();
    }

    @Override
    protected void startTurn(TurnTask task) {
        if (task == visionTurn1) {
            latestVisionAngle = -manualAngle1;
            lastDistance      = manualDistance1;
        } else if (task == visionTurn2) {
            latestVisionAngle = -manualAngle2;
            lastDistance      = manualDistance2;
        } else if (task == visionTurn3) {
            latestVisionAngle = -manualAngle3;
            lastDistance      = manualDistance3;
        }

        if (task == visionTurn1 || task == visionTurn2 || task == visionTurn3) {

            if (lastDistance==0) return;

            if (lastDistance != 0) {
                task.addWaitAction(0, motorActions.outtakeTransfer())
                        .addWaitAction(0, new SequentialAction(
                                motorActions.specimenExtend(Math.min(lastDistance * 32.25, 800)),
                                motorActions.extendo.waitUntilFinished(Math.min(lastDistance * 32.25, 800), 300),
                                motorActions.spin.eat(),
                                motorActions.inArm.sampleGrab(),
                                motorActions.inPivot.sampleGrab(),
                                motorActions.spin.eatUntilStrictSpecimen(Enums.DetectedColor.YELLOW,Enums.DetectedColor.RED, motorControl),
                                telemetryPacket -> {
                                    lastDistance=0;
                                    return false;
                                }
                        ))
                        .addWaitAction(()-> motorControl.getDetectedColor() == Enums.DetectedColor.YELLOW, motorActions.extendo.set(0));
            }

            // Now set the turn angle/side based on the stored vision angle:
            task.angle      = Math.abs(latestVisionAngle);
            task.isLeft     = (latestVisionAngle > 0);
            task.useDegrees = true;
            task.isRelative = true;

        }
        task.initiated = true;
        Pose currentRobotPose = follower.getPose();
        double currentX = currentRobotPose.getX();
        double currentY = currentRobotPose.getY();
        double currentHeadingRadians = currentRobotPose.getHeading();

        if (task.isRelative) {
            double turnAmountRadians = task.useDegrees ? Math.toRadians(task.angle) : task.angle;
            targetHeadingRadians = currentHeadingRadians + (task.isLeft ? turnAmountRadians : -turnAmountRadians);
        } else {
            targetHeadingRadians = task.useDegrees ? Math.toRadians(task.angle) : task.angle;
        }

        targetHeadingRadians = AngleUtils.normalizeRadians(targetHeadingRadians);

        PathChain turnPath = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(new Pose(currentX, currentY, currentHeadingRadians))))
                .setPathEndHeadingConstraint(0.001)
                .setConstantHeadingInterpolation(targetHeadingRadians)
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        follower.followPath(turnPath, true);

        isActivelyTurningInternalFlag = true;
    }



    @Override
    public void start() {
        latestVisionAngle = -manualAngle1;
        lastDistance      = manualDistance1;
        opModeTimer.resetTimer();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        runTasks();
        motorControl.update();


        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Task Name", tasks.get(currentTaskIndex).getClass().getSimpleName());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Turning", isTurning());
        telemetry.addData("PathActive", isPathActive());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("liftCurrent",motorControl.lift.motor.getCurrentPosition());
        telemetry.addData("liftTarget",motorControl.lift.getTargetPosition());
        telemetry.addData("isEmpty",motorControl.isEmpty());
        telemetry.addData("extendoReset",motorControl.extendo.resetting);
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.addData("Angle", latestVisionAngle);
        telemetry.addData("Distance", lastDistance);

        telemetry.update();
    }
}
