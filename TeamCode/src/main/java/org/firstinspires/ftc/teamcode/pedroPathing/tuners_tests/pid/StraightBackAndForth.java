package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Straight Back And Forth", group = "PIDF Tuning")
public class StraightBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;
    private Path backwards;
    
    // ========================================
    // TUNABLE PID CONSTANTS (Edit on Dashboard!)
    // ========================================
    
    // Translational PID (Forward/Backward, Left/Right)
    public static double translationalP = 0.15;
    public static double translationalI = 0.0;
    public static double translationalD = 0.009;
    public static double translationalF = 0.0;
    public static double translationalFF = 0.015;
    
    // Secondary Translational PID (High speed)
    public static double secondaryTranslationalP = 0.12;
    public static double secondaryTranslationalI = 0.0;
    public static double secondaryTranslationalD = 0.005;
    public static double secondaryTranslationalF = 0.0;
    
    // Heading PID (Rotation)
    public static double headingP = 3.1;
    public static double headingI = 0.0;
    public static double headingD = 0.25;
    public static double headingF = 0.0;
    
    // Secondary Heading PID
    public static double secondaryHeadingP = 2.6;
    public static double secondaryHeadingI = 0.0;
    public static double secondaryHeadingD = 0.05;
    public static double secondaryHeadingF = 0.0;
    
    // Drive PID (Overall path following)
    public static double driveP = 0.4;
    public static double driveI = 0.001;
    public static double driveD = 0.02;
    public static double driveF = 0.6;
    public static double driveG = 0.0;
    public static double driveFF = 0.02;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);


        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        // Update PID constants from dashboard values
        com.pedropathing.follower.FollowerConstants.translationalPIDFCoefficients.setCoefficients(
                translationalP, translationalI, translationalD, translationalF);
        com.pedropathing.follower.FollowerConstants.translationalPIDFFeedForward = translationalFF;
        
        com.pedropathing.follower.FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(
                secondaryTranslationalP, secondaryTranslationalI, secondaryTranslationalD, secondaryTranslationalF);
        
        com.pedropathing.follower.FollowerConstants.headingPIDFCoefficients.setCoefficients(
                headingP, headingI, headingD, headingF);
        
        com.pedropathing.follower.FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(
                secondaryHeadingP, secondaryHeadingI, secondaryHeadingD, secondaryHeadingF);
        
        com.pedropathing.follower.FollowerConstants.drivePIDFCoefficients.setCoefficients(
                driveP, driveI, driveD, driveF, driveG);
        com.pedropathing.follower.FollowerConstants.drivePIDFFeedForward = driveFF;
        
        // Update follower
        follower.update();
        
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryA.addData("going forward", forward);
        telemetryA.addData("", "");
        telemetryA.addLine("=== TUNING PID (Edit on Dashboard) ===");
        telemetryA.addData("Translational P", translationalP);
        telemetryA.addData("Translational D", translationalD);
        telemetryA.addData("Heading P", headingP);
        telemetryA.addData("Heading D", headingD);
        telemetryA.addData("Drive P", driveP);
        telemetryA.addData("", "");
        follower.telemetryDebug(telemetryA);
    }
}
