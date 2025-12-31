package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Bluegatetest")
public class Bluegatetest extends PathChainAutoOpMode {

    private Follower follower;

    private DcMotor intakefront, intakeback;
    private DcMotorEx shootr, shootl;
    private Servo reargate, launchgate, hood1, turret1, turret2;
    private Servo indexfront, indexback;

    private RobotActions actions;

    // Initial drive: Start -> Shoot
    private PathChain startToShoot;

    // Your provided intake loop paths
    private PathChain path1, path2, path3, path4;

    // =========================
    // TURRET + GOAL CONSTANTS (BLUE)
    // =========================
    public static double targetX = 16.0;
    public static double targetY = 125.0;

    public static double turretCenterPosition = 0.51;
    public static double turretLeftPosition   = 0.15;
    public static double turretRightPosition  = 0.85;
    public static double turretMaxAngle       = 135;

    public static double turretTrimDeg = 0.0;
    public static double TURRET1_BACKLASH_OFFSET = 0.015;

    private static final double TURRET_LEAD_SWITCH_T = 0.97;

    private final Set<PathChain> shooterPaths = new HashSet<>();
    private final Map<PathChain, Pose> predictedEndPoseByPath = new HashMap<>();

    // =========================
    // SHOOTER RPM (CONSTANT)
    // =========================
    private static final double SHOOT_RPM_HOLD = 1095.0;

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

        hood1.setPosition(0.48);
        launchgate.setPosition(0.5);
        reargate.setPosition(0.7);
        indexfront.setPosition(RobotActions.INDEX_FRONT_EXTENDED);
        indexback.setPosition(RobotActions.INDEX_BACK_RETRACTED);

        // BLUE turret allowed side (lower-than-center)
        turret1.setPosition(0.2);
        turret2.setPosition(0.2);

        actions = new RobotActions(
                intakefront, intakeback, shootr, shootl,
                launchgate, reargate,
                hood1, turret1, turret2,
                indexfront, indexback,
                hardwareMap.voltageSensor.iterator().next()
        );
        run(actions.safeindexer());

        // Start pose matches the Start->Shoot path start
        follower.setStartingPose(new Pose(34.032, 135.705, Math.toRadians(270)));

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        updateTurretFromPose();

        // Keep shooter RPM constant (per your standard)
        run(actions.holdShooterAtRPMclose(SHOOT_RPM_HOLD, 30));

        runTasks();

        telemetry.addData("Task", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T", follower.getCurrentTValue());
        telemetry.addData("PathBusy", follower.isBusy());
        telemetry.update();
    }

    @Override
    protected void buildPathChains() {

        // ============================================================
        // Start -> Shoot (same start/shoot coords you’ve been using for blue)
        // ============================================================
        startToShoot = follower
                .pathBuilder()
                .addPath(new BezierLine(
                        new Pose(34.032, 135.705),
                        new Pose(57.855, 84.443)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        // Treat as shooter path for turret lead-aim (optional but consistent)
        registerShooterPath_Line(startToShoot,
                34.032, 135.705,
                57.855, 84.443,
                false);

        // ============================================================
        // Your provided paths (ALL should intake)
        // ============================================================

        path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 86.000),
                        new Pose(16.804, 62.535)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .addParametricCallback(0.00, () -> run(actions.startIntake()))
                .build();

        path2 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                        new Pose(16.804, 62.535),
                        new Pose(12.762, 53.388)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .addParametricCallback(0.00, () -> run(actions.startIntake()))
                .build();

        path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                        new Pose(12.762, 53.388),
                        new Pose(9.997, 57.004)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                .addParametricCallback(0.00, () -> run(actions.startIntake()))
                .build();

        path4 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                        new Pose(9.997, 57.004),
                        new Pose(55.941, 85.932)
                ))
                .setTangentHeadingInterpolation()
                // Per your rule: NO true arg. Use setReversed() with no args.
                .setReversed()
                .addParametricCallback(0.00, () -> run(actions.startIntake()))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Drive to shoot first
        addPath(startToShoot, 0);

        // Then run the intake loop
        addPath(path1, 0);
        addPath(path2, 0);
        addPath(path3, 0);
        addPath(path4, 0);
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
        // Not used
    }

    // =========================
    // Turret math / lead aim (same structure as your template)
    // =========================

    private static double headingFromPoints(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1, x2 - x1);
    }

    private static double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private void registerShooterPath_Line(PathChain path,
                                          double xStart, double yStart,
                                          double xEnd, double yEnd,
                                          boolean reversed) {
        double tangent = headingFromPoints(xStart, yStart, xEnd, yEnd);
        double endHeading = reversed ? normalizeRadians(tangent + Math.PI) : normalizeRadians(tangent);
        shooterPaths.add(path);
        predictedEndPoseByPath.put(path, new Pose(xEnd, yEnd, endHeading));
    }

    private Pose getTurretAimPose() {
        if (follower == null) return null;

        Pose live = follower.getPose();

        if (taskPhase != 0) return live;
        if (currentTaskIndex < 0 || currentTaskIndex >= tasks.size()) return live;

        Object taskObj = tasks.get(currentTaskIndex);
        if (!(taskObj instanceof PathChainTask)) return live;

        PathChainTask pct = (PathChainTask) taskObj;
        if (!(pct.pathChain instanceof PathChain)) return live;

        PathChain activePath = (PathChain) pct.pathChain;

        if (!shooterPaths.contains(activePath)) return live;

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

        double turretAngle = normalizeRadians(angleToTargetField - currentHeading);
        double turretAngleDegrees = Math.toDegrees(turretAngle) + turretTrimDeg;

        double clampedAngle = Math.max(-turretMaxAngle, Math.min(turretMaxAngle, turretAngleDegrees));

        double servoPosition;
        if (clampedAngle >= 0) {
            double servoRange = turretRightPosition - turretCenterPosition;
            servoPosition = turretCenterPosition + (clampedAngle / turretMaxAngle) * servoRange;
        } else {
            double servoRange = turretCenterPosition - turretLeftPosition;
            servoPosition = turretCenterPosition - (Math.abs(clampedAngle) / turretMaxAngle) * servoRange;
        }

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // BLUE side: only lower-than-center
        servoPosition = Math.min(turretCenterPosition, servoPosition);

        double turret1Pos = servoPosition + TURRET1_BACKLASH_OFFSET;
        turret1Pos = Math.max(0.0, Math.min(1.0, turret1Pos));
        turret1Pos = Math.min(turretCenterPosition, turret1Pos);

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
