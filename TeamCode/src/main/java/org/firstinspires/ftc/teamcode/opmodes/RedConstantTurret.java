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

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

@Autonomous(name = "1 - RedClose18startfar")
public class RedConstantTurret extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;

    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    private PathChain path1, path2, path3, path4, path5, path6, path6_5, path7, path8, path9, path10, path11, path12, path12far, leave;

    // ----------------------------
    // FIELD MIRROR CONFIG (BLUE->RED)
    // ----------------------------
    private static final double FIELD_WIDTH = 144.0;

    private static double mirrorX(double x) {
        return FIELD_WIDTH - x;
    }

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // Heading mirror across field width: theta' = pi - theta
    private static double mirrorHeading(double headingRad) {
        return normalizeRadians(Math.PI - headingRad);
    }

    // --- Turret + goal constants (mirrored to RED) ---

    // RED target for turret aim (original RED-side coords)
    public static double targetX = 136;
    public static double targetY = 125.0;

    // RED goal zone center (original RED-side coords)
    public static double goalZoneX = 116.0;
    public static double goalZoneY = 116.0;

    // Turret servo constants
    public static double turretCenterPosition = 0.51;   // 0 deg
    public static double turretLeftPosition   = 0.15;   // max left
    public static double turretRightPosition  = 0.85;   // max right
    public static double turretMaxAngle       = 135;    // deg left/right from center

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
    private static final double SHOOT_RPM_HOLD  = 1150;

    // Hysteresis so it doesn't bounce between modes
    private static final double BOOST_EXIT_RPM  = 1150;
    private static final double BOOST_REARM_RPM = 850.0;

    private boolean shooterBoostActive = true;

    // --- Persistent override from start of Path11 through end of auto ---
    private static final double FORCED_RPM_1325 = 1325.0;
    private boolean force1325Rpm = false;

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
        hood1.setPosition(0.49);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);

        // Mirrored turret “park” (BLUE parked on lower-than-center side; RED parks on higher-than-center side)
        turret1.setPosition(0.61);
        turret2.setPosition(0.61);

        indexfront.setPosition(RobotActions.INDEX_FRONT_EXTENDED);
        indexback.setPosition(RobotActions.INDEX_BACK_RETRACTED);

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                indexfront, indexback,
                hardwareMap.voltageSensor.iterator().next()
        );

        run(actions.safeindexer());

        // Start pose mirrored from BLUE (56,8,270) -> (88,8, mirrorHeading(270deg)=270deg)
        follower.setStartingPose(new Pose(
                mirrorX(56.0),
                8.0,
                mirrorHeading(Math.toRadians(270))
        ));

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

        updateTurretFromPose();

        double vR   = shootr.getVelocity();
        double vL   = shootl.getVelocity();
        double rpmR = (vR / 28.0) * 60.0;
        double rpmL = (vL / 28.0) * 60.0;
        double avgRpm = 0.5 * (rpmR + rpmL);

        if (!force1325Rpm) {
            if (shooterBoostActive) {
                if (avgRpm >= BOOST_EXIT_RPM) shooterBoostActive = false;
            } else {
                if (avgRpm <= BOOST_REARM_RPM) shooterBoostActive = true;
            }
        } else {
            shooterBoostActive = false;
        }

        double requestedRpm = shooterBoostActive ? SHOOT_RPM_BOOST : SHOOT_RPM_HOLD;

        if (force1325Rpm) {
            requestedRpm = FORCED_RPM_1325;
        }

        run(actions.holdShooterAtRPMclose(requestedRpm, 30));

        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Override1325", force1325Rpm);
        telemetry.addData("Mode", shooterBoostActive ? "BOOST" : (force1325Rpm ? "FORCED_1325" : "HOLD"));
        telemetry.addData("Target RPM", "%.0f", requestedRpm);
        telemetry.addData("RPM R", "%.0f", rpmR);
        telemetry.addData("RPM L", "%.0f", rpmL);
        telemetry.addData("RPM avg", "%.0f", avgRpm);

        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // === Path1: mirror of BLUE path1 ===
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(56.000), 8.000),
                        new Pose(mirrorX(56.579), 87.421)))
                .setLinearHeadingInterpolation(mirrorHeading(Math.toRadians(270)),mirrorHeading(Math.toRadians(180)))
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.48)
                .addParametricCallback(0.85, () -> run(actions.launch3faster()))
                .build();

        // === Path2: mirror of BLUE path2 ===
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(56.579), 87.421),
                        new Pose(mirrorX(24), 81.000)))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path3: mirror of BLUE path3 ===
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(20.000), 79.000),
                        new Pose(mirrorX(56.792), 87.421)))
                .setTangentHeadingInterpolation().setReversed()
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.48)
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        // === Path4: mirror of BLUE path4 ===
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(mirrorX(56.792), 87.634),
                        new Pose(mirrorX(54.665), 31.000),
                        new Pose(mirrorX(12.000), 25.000)))
                .setLinearHeadingInterpolation(
                        mirrorHeading(Math.toRadians(230)),
                        mirrorHeading(Math.toRadians(180))
                )
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path5: mirror of BLUE path5 ===
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(mirrorX(9.000), 25.000),
                        new Pose(mirrorX(54.665), 35.734),
                        new Pose(mirrorX(56.579), 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.48)
                .addParametricCallback(0.85, () -> run(actions.launch3faster()))
                .build();

        // === Path6: mirror of BLUE path6 ===
        path6 = follower
                .pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(mirrorX(56.000), 87.000),
                        new Pose(mirrorX(55.090), 59.557),
                        new Pose(mirrorX(14), 57)
                ))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path6.5: mirror of BLUE path6_5 ===
        path6_5 = follower
                .pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(mirrorX(10), 57),
                        new Pose(mirrorX(24), 62.109),
                        new Pose(mirrorX(16.165), 69.129)
                ))
                .setLinearHeadingInterpolation(
                        mirrorHeading(Math.toRadians(180)),
                        mirrorHeading(Math.toRadians(270))
                )
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path7: mirror of BLUE path7 ===
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(mirrorX(16.165), 69.129),
                        new Pose(mirrorX(24.000), 50.000),
                        new Pose(mirrorX(56.792), 87.634)))
                .setLinearHeadingInterpolation(
                        mirrorHeading(Math.toRadians(270)),
                        mirrorHeading(Math.toRadians(200))
                )
                .setTValueConstraint(0.96)
                .setGlobalDeceleration(0.48)
                .addParametricCallback(0.85, () -> run(actions.launch3faster()))
                .build();

        // === Path8: mirror of BLUE path8 ===
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(mirrorX(53.176), 89.335),
                        new Pose(mirrorX(13.000), 62.109),
                        new Pose(mirrorX(9.000), 51.261),
                        new Pose(mirrorX(7.000), 10.00)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, () -> run(actions.startIntake()))
                .build();

        // === Path9: mirror of BLUE path9 ===
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(11.486), 11.273),
                        new Pose(mirrorX(56.579), 87.634)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.65)
                .addParametricCallback(0.85, () -> run(actions.launch3faster()))
                .build();

        // === Path11: mirror of BLUE path11 (intake + latch forced RPM) ===
        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(mirrorX(56.579), 87.634),
                        new Pose(mirrorX(70.000), 8.000),
                        new Pose(mirrorX(12.000), 8.000)
                ))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration(0.48)
                .addParametricCallback(0, () -> {
                    run(actions.startIntake());
                    force1325Rpm = true;
                    shooterBoostActive = false;
                })
                .build();

        // === Path12: mirror of BLUE path12 ===
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(20.000), 11.273),
                        new Pose(mirrorX(62.000), 97.000)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(0.7)
                .addParametricCallback(0.9, () -> run(actions.launch3faster()))
                .build();

        // === Path12far: mirror of BLUE path12far ===
        path12far = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(12.000), 8.000),
                        new Pose(mirrorX(57.000), 13.000)))
                .setConstantHeadingInterpolation(mirrorHeading(Math.toRadians(180)))
                .setGlobalDeceleration(0.43)
                .addParametricCallback(0.9, () -> run(new SequentialAction(
                        new SleepAction(0.001),
                        actions.launch3far()
                )))
                .build();

        // === leave: mirror of BLUE leave ===
        leave = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(mirrorX(57.000), 13.000),
                        new Pose(mirrorX(12.000), 8.000)))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        // --- Register shooter paths (mirrored) + predicted end poses ---
        // Path1 uses LinearHeadingInterpolation -> predict with explicit end heading
        registerShooterPath_WithEndHeading(
                path1,
                mirrorX(56.579), 87.421,
                mirrorHeading(Math.toRadians(180))   // end heading of path1
        );


        registerShooterPath_Line(path3,
                mirrorX(20.000), 79.000,
                mirrorX(56.792), 87.421, true);

        registerShooterPath_CurveEnd(path5,
                mirrorX(54.665), 35.734,
                mirrorX(56.579), 87.634,
                mirrorX(56.579), 87.634, true);

        // UPDATED: Path7 uses LinearHeadingInterpolation, so predicted heading must be the path's end heading,
        // not the path tangent.
        registerShooterPath_WithEndHeading(
                path7,
                mirrorX(56.792), 87.634,
                mirrorHeading(Math.toRadians(200))
        );

        registerShooterPath_Line(path9,
                mirrorX(11.486), 11.273,
                mirrorX(56.579), 87.634, true);

        registerShooterPath_Line(path12,
                mirrorX(20.000), 11.273,
                mirrorX(62.000), 97.000, true);
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        PathChainTask path1Task = new PathChainTask(path1, 0.8);
        tasks.add(path1Task);

        addPath(path2, 0);

        PathChainTask path3Task = new PathChainTask(path3, 0.8);
        tasks.add(path3Task);

        addPath(path4, 0);

        PathChainTask path5Task = new PathChainTask(path5, 0.8);
        tasks.add(path5Task);

        addPath(path6, 0);
        addPath(path6_5, 0.2);

        PathChainTask path7Task = new PathChainTask(path7, 0.8);
        tasks.add(path7Task);

        addPath(path8, 0);

        PathChainTask path9Task = new PathChainTask(path9, 0.8);
        tasks.add(path9Task);

        addPath(path11, 0);

        PathChainTask path12Task = new PathChainTask(path12far, 0.8);
        tasks.add(path12Task);

        addPath(leave, 0);
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
        follower.followPath((PathChain) task.pathChain, true);
    }

    @Override
    protected void startTurn(TurnTask task) {
        // Not used in this auto
    }

    // -----------------------------
    // Turret lead-aim implementation (UPDATED for Linear/Constant heading paths)
    // -----------------------------

    private static double headingFromPoints(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1, x2 - x1);
    }

    // Use for paths whose robot heading follows the path tangent (tangent heading interpolation)
    private void registerShooterPath_Line(PathChain path, double xStart, double yStart,
                                          double xEnd, double yEnd, boolean reversed) {
        double tangent = headingFromPoints(xStart, yStart, xEnd, yEnd);
        double endHeading = reversed ? normalizeRadians(tangent + Math.PI) : normalizeRadians(tangent);
        shooterPaths.add(path);
        predictedEndPoseByPath.put(path, new Pose(xEnd, yEnd, endHeading));
    }

    // Use for curves whose robot heading follows the curve tangent (tangent heading interpolation)
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

    // NEW: Use when the path uses LinearHeadingInterpolation or ConstantHeadingInterpolation.
    // Tangent-at-end is NOT necessarily the robot heading, so we store the explicit end heading.
    private void registerShooterPath_WithEndHeading(PathChain path,
                                                    double xEnd, double yEnd,
                                                    double endHeadingRad) {
        shooterPaths.add(path);
        predictedEndPoseByPath.put(path, new Pose(xEnd, yEnd, normalizeRadians(endHeadingRad)));
    }

    private Pose getTurretAimPose() {
        if (follower == null) return null;

        Pose live = follower.getPose();

        // Only predict while actively driving a path
        if (taskPhase != 0) return live;

        if (currentTaskIndex < 0 || currentTaskIndex >= tasks.size()) return live;

        Object taskObj = tasks.get(currentTaskIndex);
        if (!(taskObj instanceof PathChainTask)) return live;

        PathChainTask pct = (PathChainTask) taskObj;
        if (!(pct.pathChain instanceof PathChain)) return live;

        PathChain activePath = (PathChain) pct.pathChain;

        // Only predict for shooter paths
        if (!shooterPaths.contains(activePath)) return live;

        // Predict early in the path, then use live near the end
        double t = follower.getCurrentTValue();
        if (t < TURRET_LEAD_SWITCH_T) {
            Pose predicted = predictedEndPoseByPath.get(activePath);
            if (predicted != null) return predicted;
        }

        return live;
    }

    private void updateTurretFromPose() {
        if (follower == null || turret1 == null || turret2 == null) return;

        Pose aimPose = getTurretAimPose();
        if (aimPose == null) return;

        double currentX = aimPose.getX();
        double currentY = aimPose.getY();
        double currentHeading = aimPose.getHeading();

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        double distanceToGoalInches = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        double angleToTargetField = Math.atan2(deltaY, deltaX);

        double turretAngle = angleToTargetField - currentHeading;
        turretAngle = normalizeRadians(turretAngle);

        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;

        double clampedAngle = Math.max(-turretMaxAngle,
                Math.min(turretMaxAngle, turretAngleDegrees));

        double servoPosition;
        if (clampedAngle >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // MIRRORED CONSTRAINT: RED side is the opposite of BLUE.
        // BLUE limited to <= center (lower-than-center). RED limits to >= center (higher-than-center).
        servoPosition = Math.max(turretCenterPosition, servoPosition);

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));
        turret1Pos = Math.max(turretCenterPosition, turret1Pos);

        turret1.setPosition(turret1Pos - 0.01);
        turret2.setPosition(servoPosition - 0.01);

        telemetry.addData("Turret dist (in)", "%.1f", distanceToGoalInches);
        telemetry.addData("Turret angle (deg)", "%.1f", turretAngleDegrees);
        telemetry.addData("Turret servo", "%.3f", servoPosition);

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
