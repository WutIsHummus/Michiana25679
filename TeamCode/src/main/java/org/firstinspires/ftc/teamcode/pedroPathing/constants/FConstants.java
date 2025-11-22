package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;


        FollowerConstants.leftFrontMotorName = "frontleft";
        FollowerConstants.leftRearMotorName = "backleft";
        FollowerConstants.rightFrontMotorName = "frontright";
        FollowerConstants.rightRearMotorName = "backright";
        FollowerConstants.useBrakeModeInTeleOp = true;

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13.222218;

        FollowerConstants.xMovement = 77.0763;
        FollowerConstants.yMovement = 62.6258;

        FollowerConstants.forwardZeroPowerAcceleration = -33.0504;
        FollowerConstants.lateralZeroPowerAcceleration = -68.7352;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.08,
                0,
                0.001,
                0);
        FollowerConstants.translationalPIDFFeedForward = 0.00;

        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.12,
                0,
                0.005,0);

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0,0.005,0);

        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2.6,0,0.05,0);

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.004 ,0,0.001,0.6,0);
        FollowerConstants.drivePIDFFeedForward = 0.02;


        FollowerConstants.useSecondaryDrivePID = false   ;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.32,0,0.005,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4.75;
        FollowerConstants.centripetalScaling = 0.00025;

        FollowerConstants.pathEndTimeoutConstraint = 75;
        FollowerConstants.pathEndTValueConstraint = 0.96;
        FollowerConstants.pathEndVelocityConstraint = 3.2;
        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.useVoltageCompensationInTeleOp = true;
        FollowerConstants.nominalVoltage = 13.7 ;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
