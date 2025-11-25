package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Constants class for PedroPathing 2.0
 * This class configures all robot-specific settings for the path follower.
 * Values migrated from FConstants.java and LConstants.java
 */
public class Constants {

    // Follower constants - PID coefficients, acceleration, and path following parameters
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.222218)
            .forwardZeroPowerAcceleration(-32.4139)
            .lateralZeroPowerAcceleration(-71.0266)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.004, 0.0025))

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.04, 0.02))
            //.headingPIDFSwitch(20)
            //.useSecondaryHeadingPIDF(false)
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1,0,0.04,0.02))
            // Note: Original had useSecondaryHeadingPID = false, but PedroPathing 2.0 requires switch value
            // Setting to very high value (999) to effectively disable secondary PID (original behavior)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0.00, 0.0001 , 0.6, 0.03))
            //primary is fast at 0.01 p and 0.01d, but testing with 0.005 p and 0.001d
            .useSecondaryDrivePIDF(false)
            //.drivePIDFSwitch(20)
            // p secondary works at 0.005, testing a higher value
            //.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005,0,0.001,0.6,0.01))
            // Note: Original had useSecondaryDrivePID = false, but PedroPathing 2.0 requires switch value
            // Setting to very high value (999) to effectively disable secondary PID (original behavior)
            .centripetalScaling(0.0005);

    // Drivetrain constants - motor names, directions, and velocities
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontleft")
            .leftRearMotorName("backleft")
            .rightFrontMotorName("frontright")
            .rightRearMotorName("backright")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(74.431)
            .yVelocity(59.044);

    // Localizer constants - Pinpoint odometry pod configuration
    // Original values: forwardY=7.5, strafeX=0, hardwareMapName="pinpoint"
    // Original had useCustomEncoderResolution = false, so using default encoder resolution
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.5)
            .strafePodX(0)
            .hardwareMapName("pinpoint")  // Hardware map name for Pinpoint odometry pods
            // Not setting customEncoderResolution since original had useCustomEncoderResolution = false
            // If you need custom resolution, uncomment: .customEncoderResolution(13.26291192)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /**
     * PathConstraints constructor parameters in order:
     * tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint,
     * timeoutConstraint, brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart
     *
     * BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10
     */
    public static PathConstraints pathConstraints = new PathConstraints(

            0.98,      // tValueConstraint - reduced from 0.99 to stop earlier and prevent overshoot
            0.1,       // velocityConstraint - increased from 1 to allow more reasonable stopping velocity
            0.08,      // translationalConstraint - tightened from 0.1 to stop more precisely at path end
            0.007,     // headingConstraint (from pathEndHeadingConstraint = 0.007)
            25,        // timeoutConstraint (from pathEndTimeoutConstraint = 75)
            0.5,       // brakingStrength - increased from 1 to 2.5 for more aggressive braking to prevent overshoot
            10,        // BEZIER_CURVE_SEARCH_LIMIT (should not be changed)
            1        // brakingStart - increased from 1 to 2.0 to start braking earlier and prevent overshoot
    );

    /**
     * Creates a Follower instance configured with all robot-specific settings
     *
     * @param hardwareMap The hardware map from the OpMode
     * @return A configured Follower instance ready to use
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
