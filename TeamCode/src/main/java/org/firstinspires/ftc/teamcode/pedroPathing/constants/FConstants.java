package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;


        FollowerConstants.leftFrontMotorName = "FL";
        FollowerConstants.leftRearMotorName = "BL";
        FollowerConstants.rightFrontMotorName = "FR";
        FollowerConstants.rightRearMotorName = "BR";
        FollowerConstants.useBrakeModeInTeleOp = true;

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 14.787;

        FollowerConstants.xMovement = 72.15606995309422;
        FollowerConstants.yMovement = 47.90792618802868;

        FollowerConstants.forwardZeroPowerAcceleration = -44.09575584757384;
        FollowerConstants.lateralZeroPowerAcceleration = -83.99702874916117;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15,
                0,
                0.009,
                0);
        FollowerConstants.translationalPIDFFeedForward = 0.015;

        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.12,
                0,
                0.005,0);

        FollowerConstants.headingPIDFCoefficients.setCoefficients(3,0,0.25,0);

        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(3.3,0,0.01,0);

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.4 ,0.001,0.02,0.6,0);
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
        FollowerConstants.nominalVoltage = 13.2 ;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
