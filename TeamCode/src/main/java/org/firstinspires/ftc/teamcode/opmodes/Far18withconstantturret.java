package org.firstinspires.ftc.teamcode.opmodes;

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

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

@Autonomous(name = "1 - BlueClose18startfar")
public class Far18withconstantturret extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12;

    // --- Turret + goal constants (copied from Blue teleop) ---

    // Field target for turret aim (mirrored blue-side coords)
    public static double targetX = 144.0 - 128.0; // 16.0
    public static double targetY = 125.0;

    // Goal “zone” center (for distance telemetry if you want)
    public static double goalZoneX = 144.0 - 116.0; // 28.0
    public static double goalZoneY = 116.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;  // max left
    public static double turretRightPosition  = 0.85;  // max right
    public static double turretMaxAngle       = 135;   // deg left/right from center

    // Trim + backlash
    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.015;

    // --- Lead-aim config ---
    private static final double TURRET_LEAD_SWITCH_T = 0.95;

    // Paths that end with launch3faster(), and their predicted end poses
    private final Set<PathChain> shooterPaths = new HashSet<>();
    private final Map<PathChain, Pose> predictedEndPoseByPath = new HashMap<>();

    // --- Shooter two-stage setpoint (spin-up boost then settle) ---
    private static final double SHOOT_RPM_BOOST = 1700;
    private static final double SHOOT_RPM_HOLD  = 1080.0;

    // Hysteresis so it doesn't bounce between modes
    private static final double BOOST_EXIT_RPM  = 1080.0; // switch to HOLD when above this
    private static final double BOOST_REARM_RPM = 850.0;  // if you drop below this, allow BOOST again

    private boolean shooterBoostActive = true; // start in boost so you get up to speed quickly

    @Override
    public void init() {
        super.init();  // important so PathChainAutoOpMode can init its state

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

        // Shooter motor direction
        shootl.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Servo init
        hood1.setPosition(0.48);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);
        turret1.setPosition(0.175);
        turret2.setPosition(0.15);

        actions = new RobotActions(intakefront, intakeback, shootr, shootl,
                launchgate, reargate, turret1, turret2, hood1);

        // Start pose matches Path1 start
        follower.setStartingPose(new Pose(56.0, 8.0, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();  // important so task state is reset correctly
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        // Lead-aim turret: predicted end pose for first 90% on shooter paths, live correction last 10%
        updateTurretFromPose();

        // --- Shooter two-stage target: BOOST -> HOLD based on measured RPM ---
        double vR   = shootr.getVelocity();
        double vL   = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avgRpm = 0.5 * (rpmR + rpmL);

        // Latch with hysteresis
        if (shooterBoostActive) {
            if (avgRpm >= BOOST_EXIT_RPM) shooterBoostActive = false;   // BOOST -> HOLD
        } else {
            if (avgRpm <= BOOST_REARM_RPM) shooterBoostActive = true;   // HOLD -> BOOST (re-arm)
        }

        double requestedRpm = shooterBoostActive ? SHOOT_RPM_BOOST : SHOOT_RPM_HOLD;

        // Keep shooter spun up (your existing action)
        run(actions.holdShooterAtRPMclose(requestedRpm, 30));

        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Mode", shooterBoostActive ? "BOOST" : "HOLD");
        telemetry.addData("Target RPM", "%.0f", requestedRpm);
        telemetry.addData("RPM R", "%.0f", rpmR);
        telemetry.addData("RPM L", "%.0f", rpmL);
        telemetry.addData("RPM avg", "%.0f", avgRpm);

        telemetry.update();
    }

    @Override
    protected void buildPathChains() {
        // Shooter heading we want to hold at the end (unused for tangent paths, kept for reference)
        double shooterHeading = Math.toRadians(215);

        // === Path1: (56,8) -> (56.579,87.421), then settle at shooter pose ===
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(56.579, 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        // === Path2: (56.579,87.421) -> (18,79), intake path ===
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.579, 87.421),
                        new Pose(18, 81)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path3: (20,79) -> (56.792,87.421), then settle at shooter pose ===
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20, 79),
                        new Pose(56.792, 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.792, 87.634),
                        new Pose(54.665, 31),
                        new Pose(12, 25)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(9, 25),
                        new Pose(54.665, 35.734),
                        new Pose(56.579, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.000, 87.000),
                        new Pose(39.988, 52.538),
                        new Pose(19, 57),
                        new Pose(14, 60)))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(12, 58),
                        new Pose(24, 50),
                        new Pose(56.792, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.45)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(53.176, 89.335),
                        new Pose(13, 62.109),
                        new Pose(9, 51.261),
                        new Pose(7, 12)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(11.486, 11.273),
                        new Pose(56.579, 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.579, 87.634),
                        new Pose(70, 11),
                        new Pose(12, 8.721)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20, 11.273),
                        new Pose(62, 97)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.6)
                .addParametricCallback(0.7, () -> run(actions.launch3faster()))
                .build();

        // --- Register shooter paths (launch3faster paths) + predicted end poses ---
        // All of these were built with .setReversed(), so predicted heading = tangent + PI.

        registerShooterPath_Line(path1, 56.000, 8.000, 56.579, 87.421, true);
        registerShooterPath_Line(path3, 20.000, 79.000, 56.792, 87.421, true);

        // BezierCurve end tangent uses (end - lastControl)
        registerShooterPath_CurveEnd(path5, 54.665, 35.734, 56.579, 87.634, 56.579, 87.634, true);
        registerShooterPath_CurveEnd(path7, 24.000, 50.000, 56.792, 87.634, 56.792, 87.634, true);

        registerShooterPath_Line(path9, 11.486, 11.273, 56.579, 87.634, true);
        registerShooterPath_Line(path12, 20.000, 11.273, 62.000, 97, true);
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Shoot after Path1 (preload)
        PathChainTask path1Task = new PathChainTask(path1, 0.7);
        tasks.add(path1Task);

        // Path2: intake only
        addPath(path2, 0);

        // Shoot after Path3
        PathChainTask path3Task = new PathChainTask(path3, 0.7);
        tasks.add(path3Task);

        // Path4: intake only
        addPath(path4, 0);

        // Shoot after Path5
        PathChainTask path5Task = new PathChainTask(path5, 0.7);
        tasks.add(path5Task);

        // Path6: intake only
        addPath(path6, 0);

        // Shoot after Path7
        PathChainTask path7Task = new PathChainTask(path7, 0.7);
        tasks.add(path7Task);

        // Path8: intake only
        addPath(path8, 0);

        // Shoot after Path9
        PathChainTask path9Task = new PathChainTask(path9, 0.7);
        tasks.add(path9Task);

        // Path11: intake only
        addPath(path11, 0);

        // Shoot after Path12
        PathChainTask path12Task = new PathChainTask(path12, 3);
        tasks.add(path12Task);
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
        follower.followPath((PathChain) task.pathChain, true); // holdEnd = true
    }

    @Override
    protected void startTurn(TurnTask task) {
        // Not used in this auto
    }

    // -----------------------------
    // Turret lead-aim implementation
    // -----------------------------

    private static double headingFromPoints(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1, x2 - x1);
    }

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private void registerShooterPath_Line(PathChain path, double xStart, double yStart,
                                          double xEnd, double yEnd, boolean reversed) {
        double tangent = headingFromPoints(xStart, yStart, xEnd, yEnd);
        double endHeading = reversed ? normalizeRadians(tangent + Math.PI) : normalizeRadians(tangent);
        shooterPaths.add(path);
        predictedEndPoseByPath.put(path, new Pose(xEnd, yEnd, endHeading));
    }

    /**
     * For BezierCurve, end tangent direction is (end - lastControl).
     * You also pass the actual end point so the stored Pose is correct.
     */
    private void registerShooterPath_CurveEnd(PathChain path,
                                              double xLastControl, double yLastControl,
                                              double xEnd, double yEnd,
                                              double poseXEnd, double poseYEnd,
                                              boolean reversed) {
        double tangent = headingFromPoints(xLastControl, yLastControl, xEnd, yEnd);
        double endHeading = reversed ? normalizeRadians(tangent + Math.PI) : normalizeRadians(tangent);
        shooterPaths.add(path);
        predictedEndPoseByPath.put(path, new Pose(poseXEnd, poseYEnd, endHeading));
    }

    /**
     * Returns the pose the turret should use for aiming right now:
     * - For shooter paths (launch3faster paths): use predicted end pose while t < 0.90
     * - Otherwise: use live follower pose
     */
    private Pose getTurretAimPose() {
        if (follower == null) return null;

        Pose live = follower.getPose();

        // Only “lead aim” while actively driving the current path
        if (taskPhase != 0) return live;

        // Defensive checks against task list state
        if (currentTaskIndex < 0 || currentTaskIndex >= tasks.size()) return live;

        Object taskObj = tasks.get(currentTaskIndex);
        if (!(taskObj instanceof PathChainTask)) return live;

        PathChainTask pct = (PathChainTask) taskObj;
        if (!(pct.pathChain instanceof PathChain)) return live;

        PathChain activePath = (PathChain) pct.pathChain;

        // Only apply to shooter paths
        if (!shooterPaths.contains(activePath)) return live;

        // Use t-based hard switch
        double t = follower.getCurrentTValue();
        if (t < TURRET_LEAD_SWITCH_T) {
            Pose predicted = predictedEndPoseByPath.get(activePath);
            if (predicted != null) return predicted;
        }

        return live;
    }

    /**
     * Uses Pedro pose (either predicted or live depending on t) to:
     *  - compute angle to goal
     *  - compute distance to goal
     *  - set turret1/turret2 servo positions accordingly
     */
    private void updateTurretFromPose() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getTurretAimPose();
        if (aimPose == null) return;

        // Current robot pose used for aiming
        double currentX = aimPose.getX();
        double currentY = aimPose.getY();
        double currentHeading = aimPose.getHeading(); // radians

        // Vector from robot to turret target
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        // Distance to goal (inches)
        double distanceToGoalInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Field angle to target (0 rad = +X, CCW positive)
        double angleToTargetField = Math.atan2(deltaY, deltaX);

        // Angle of target relative to robot heading (what turret must turn)
        double turretAngle = angleToTargetField - currentHeading;
        turretAngle = normalizeRadians(turretAngle);

        // Convert to degrees and apply trim
        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;

        // Clamp to allowed turret range
        double clampedAngle = Math.max(-turretMaxAngle,
                Math.min(turretMaxAngle, turretAngleDegrees));

        // Map angle → servo position
        double servoPosition;
        if (clampedAngle >= 0) {
            // Positive angle = turn right
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            // Negative angle = turn left
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        // Clamp servo range
        // Clamp servo range
        // Clamp servo range
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

// RIGHT-ONLY LIMIT (your turret's right is LOWER than center)
        servoPosition = Math.min(turretCenterPosition, servoPosition);

// Backlash compensation
        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));

// RIGHT-ONLY LIMIT for turret1
        turret1Pos = Math.min(turretCenterPosition, turret1Pos);

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);



        // Telemetry
        telemetry.addData("Turret dist (in)", "%.1f", distanceToGoalInches);
        telemetry.addData("Turret angle (deg)", "%.1f", turretAngleDegrees);
        telemetry.addData("Turret servo", "%.3f", servoPosition);

        // Debug switch behavior
        double t = follower.getCurrentTValue();
        boolean predict = (taskPhase == 0 && t < TURRET_LEAD_SWITCH_T);
        telemetry.addData("TurretAimMode", predict ? "PREDICT" : "LIVE");
        telemetry.addData("TurretAimT", "%.2f", t);
    }

    @Override
    public void stop() {
        try {
            PoseStore.save(follower.getPose());
        } catch (Exception ignored) {}
        super.stop();
    }
}
