package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.hardware.RobotActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "1 - BlueClose18startfarnewgate")
public class BlueClose18startfarnewgate extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path6_5, path7, path8, path9, path10, path11;
    private PathChain path12far, leave;

    // =========================
    // Turret target (BLUE)
    // =========================
    public static double targetX = 144.0 - 122; // 19.0 (note: comment said 16.0, but expression is 19.0)
    public static double targetY = 125.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 140.0;  // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.025;

    // Live tracking after this
    private static final double TURRET_LIVE_T = 0.95;

    // Shooter setpoints
    private static final double BASE_TARGET_RPM   = 1090;
    private static final double LATE_TARGET_RPM   = 1390;

    // Boost logic
    private static final double SHOOT_RPM_BOOST = 1700.0;
    private static final double BOOST_EXIT_RPM  = 1090;
    private static final double BOOST_REARM_RPM = 850.0;

    private boolean shooterBoostActive = true;
    private boolean afterPath10 = false; // becomes true via callback at start of path10

    // Deceleration settings
    private static final double GLOBAL_DECEL = 0.57;
    private static final double PATH12FAR_DECEL = 0.46;

    // =========================
    // New Path 6.5 (from your screenshot)
    // End Point: (15.74002954, 69.341211225)
    // Control Point 1: (18.71787296, 62.3220088)
    // Heading: Linear 180 -> 270
    // =========================
    private static final double P65_END_X  = 15.74002954;
    private static final double P65_END_Y  = 69.341211225;
    private static final double P65_C1_X   = 18.71787296;
    private static final double P65_C1_Y   = 62.3220088;

    @Override
    public void init() {
        super.init();

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

        indexfront  = hardwareMap.get(Servo.class, "indexfront");
        indexback   = hardwareMap.get(Servo.class, "indexback");

        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servo init
        hood1.setPosition(0.475);
        turret1.setPosition(0.175);
        turret2.setPosition(0.175);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);

        // Indexer init to known safe baseline
        indexfront.setPosition(RobotActions.INDEX_FRONT_RETRACTED);
        indexback.setPosition(RobotActions.INDEX_BACK_EXTENDED);

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                indexfront, indexback,
                hardwareMap.voltageSensor.iterator().next()
        );

        // Required
        run(actions.safeindexer());

        // Start pose
        follower.setStartingPose(new Pose(56.0, 8.0, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // Turret aiming (predictive until t>=0.95 on shoot paths)
        updateTurret();

        // Shooter RPM boost/hold around whichever target is active
        double vR   = shootr.getVelocity();
        double vL   = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avgRpm = 0.5 * (rpmR + rpmL);

        double targetRpm = afterPath10 ? LATE_TARGET_RPM : BASE_TARGET_RPM;

        if (shooterBoostActive) {
            if (avgRpm >= BOOST_EXIT_RPM) shooterBoostActive = false;
        } else {
            if (avgRpm <= BOOST_REARM_RPM) shooterBoostActive = true;
        }

        double requestedRpm = shooterBoostActive ? SHOOT_RPM_BOOST : targetRpm;

        run(actions.holdShooterAtRPMclose(requestedRpm, 30));

        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("afterPath10", afterPath10);
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Mode", shooterBoostActive ? "BOOST" : "HOLD");
        telemetry.addData("Requested RPM", "%.0f", requestedRpm);
        telemetry.addData("RPM avg", "%.0f", avgRpm);
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // PATH 1 (shoot path) - limit velocity to 20 + shoot at 0.95
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(56.579, 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .setVelocityConstraint(20) // requested: limit velocity on path 1 to 20
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .addParametricCallback(0.95, () -> run(new SequentialAction(
                        new SleepAction(0.1),
                        actions.launch3faster()
                )))                .build();

        // PATH 2 (intake)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.421),
                        new Pose(20, 80)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 3 (shoot path)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20, 82),
                        new Pose(56.792, 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.85, () -> run(actions.launch3faster()))
                .build();

        // PATH 4 (intake)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.792, 87.634),
                        new Pose(54.665, 29),
                        new Pose(14, 28)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 5 (shoot path)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(10, 25),
                        new Pose(54.665, 35.734),
                        new Pose(56.579, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.6)
                .setGlobalDeceleration(0.55)
                .addParametricCallback(0.85, () -> run(actions.launch3faster()))
                .build();

        // PATH 6 (intake)
        // CHANGE: make Path6 end at the FIRST pose of Path6.5 (Pose(12, 52)) AND end heading = 180
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.792, 87.421),
                        new Pose(50, 48),
                        new Pose(14, 52)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .setTValueConstraint(0.9)
                .setNoDeceleration()// end heading matches Path6.5 start
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 6.5 (NEW) - intake path
        // Start pose = end of Path6: (12, 52), heading starts at 180 and ends at 270
        path6_5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(14, 52),
                        new Pose(22, 62),
                        new Pose(17, 66)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .setNoDeceleration()
                .setTValueConstraint(0.9)
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 7 (shoot path)
        // CHANGE: start at END pose+heading of Path6.5, and set heading interpolation linear 270 -> 230
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(17, 66),
                        new Pose(53.176, 89.335)))
                //.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(230))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(GLOBAL_DECEL)
                .addParametricCallback(0.95, () -> run(actions.launch3faster()))
                .build();

        // PATH 8 (intake)
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(53.176, 89.335),
                        new Pose(13, 62.109),
                        //new Pose(9, 51.261),
                        new Pose(9, 20)
                ))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> run(actions.startIntake()))
                .build();

        // PATH 9 (shoot path)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(9, 20),
                        new Pose(56.579, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.55)
                .addParametricCallback(0.95, () -> run(actions.launch3faster()))
                .build();

        // PATH 10 (intake) - at t=0, latch RPM change flag
        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.579, 87.634),
                        new Pose(70, 10),
                        new Pose(15, 8.721)
                ))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addParametricCallback(0.0, () -> {
                    afterPath10 = true;
                    targetX = 144.0 - 125;
                    hood1.setPosition(0.45);
                    run(actions.startIntake());
                })
                .build();

        // REPLACE path12 with: path12far + leave
        path12far = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15, 8.721),
                        new Pose(57, 13)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setGlobalDeceleration(PATH12FAR_DECEL)
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(57, 13),
                        new Pose(50, 18)))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration(0.5)
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        final double WAIT_AFTER_SHOOT = 0.8;

        tasks.add(new PathChainTask(path1, 1));
        addPath(path2, 0.0);

        tasks.add(new PathChainTask(path3, WAIT_AFTER_SHOOT));
        addPath(path4, 0.0);

        tasks.add(new PathChainTask(path5, WAIT_AFTER_SHOOT));
        addPath(path6, 0.0);

        // NEW: Path 6.5 inserted between Path6 and Path7 (intake path)
        addPath(path6_5, 0.0);

        tasks.add(new PathChainTask(path7, WAIT_AFTER_SHOOT));
        addPath(path8, 0.0);

        tasks.add(new PathChainTask(path9, WAIT_AFTER_SHOOT));
        addPath(path10, 0.0);

        tasks.add(new PathChainTask(path12far, 1));
        addPath(leave, 0.0);
    }

    @Override
    protected boolean isPathActive() { return follower.isBusy(); }

    @Override
    protected boolean isTurning() { return false; }

    @Override
    protected double getCurrentTValue() { return follower.getCurrentTValue(); }

    @Override
    protected void startPath(PathChainTask task) {
        follower.followPath((PathChain) task.pathChain, true); // holdEnd = true
    }

    @Override
    protected void startTurn(TurnTask task) { }

    // =========================
    // Minimal predictive turret:
    // - For shoot paths: use end-of-path pose until t>=0.95, then live
    // - Includes clamp (angle clamp to turretMaxAngle)
    // =========================
    private void updateTurret() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getAimPoseForTurret();

        double currentX = aimPose.getX();
        double currentY = aimPose.getY();
        double currentHeading = aimPose.getHeading();

        double dx = targetX - currentX;
        double dy = targetY - currentY;

        double angleToTargetField = Math.atan2(dy, dx);
        double turretAngle = normalizeRadians(angleToTargetField - currentHeading);

        double turretDeg = Math.toDegrees(turretAngle) + turretTrimDeg;

        // Keep clamp
        double clampedDeg = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretDeg));

        double servoPosition;
        if (clampedDeg >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedDeg / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedDeg) / turretMaxAngle) * servoRange;
        }

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);
    }

    private Pose getAimPoseForTurret() {
        Pose live = follower.getPose();

        if (taskPhase != 0) return live;

        double t = follower.getCurrentTValue();
        if (t >= TURRET_LIVE_T) return live;

        PathChain active = getActivePathIfAny();
        if (active == null) return live;

        // Shoot paths in THIS opmode: 1,3,5,7,9,12far
        if (active == path1) return poseAtEndOfPath1();
        if (active == path3) return poseAtEndOfPath3();
        if (active == path5) return poseAtEndOfPath5();
        if (active == path7) return poseAtEndOfPath7(); // UPDATED for new Path7 geometry/heading
        if (active == path9) return poseAtEndOfPath9();
        if (active == path12far) return poseAtEndOfPath12far();

        return live;
    }

    private PathChain getActivePathIfAny() {
        if (currentTaskIndex < 0 || currentTaskIndex >= tasks.size()) return null;
        Object taskObj = tasks.get(currentTaskIndex);
        if (!(taskObj instanceof PathChainTask)) return null;
        PathChainTask pct = (PathChainTask) taskObj;
        if (!(pct.pathChain instanceof PathChain)) return null;
        return (PathChain) pct.pathChain;
    }

    // --- Predicted end poses ---
    private Pose poseAtEndOfPath1() {
        double h = headingFromLine(56.000, 8.000, 56.579, 87.421, true);
        return new Pose(56.579, 87.421, h);
    }

    private Pose poseAtEndOfPath3() {
        double h = headingFromLine(20.0, 82.0, 56.792, 87.421, true);
        return new Pose(56.792, 87.421, h);
    }

    private Pose poseAtEndOfPath5() {
        double h = headingFromLine(54.665, 35.734, 56.579, 87.634, true);
        return new Pose(56.579, 87.634, h);
    }

    private Pose poseAtEndOfPath7() {
        // Path7 is now a LINE from Path6.5 end -> (56.792,87.634) with linear heading ending at 230.
        // For predictive aim, use the end pose and the END heading (230deg) directly.
        return new Pose(56.792, 87.634, Math.toRadians(230));
    }

    private Pose poseAtEndOfPath9() {
        double h = headingFromLine(11.486, 11.273, 56.579, 87.634, true);
        return new Pose(56.579, 87.634, h);
    }

    private Pose poseAtEndOfPath12far() {
        return new Pose(57.0, 13.0, Math.toRadians(180));
    }

    private static double headingFromLine(double x1, double y1, double x2, double y2, boolean reversed) {
        double tangent = Math.atan2(y2 - y1, x2 - x1);
        double h = reversed ? (tangent + Math.PI) : tangent;
        return normalizeRadians(h);
    }

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    @Override
    public void stop() {
        try { PoseStore.save(follower.getPose()); } catch (Exception ignored) {}
        super.stop();
    }
}
