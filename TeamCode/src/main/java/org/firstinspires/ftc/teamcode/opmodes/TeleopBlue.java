package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Blue Teleop", group = "Competition")
public class TeleopBlue extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private DcMotor intakefront, intakeback;
    private DcMotor shootr, shootl;

    private Servo reargate;
    private Servo launchgate;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "frontleft");
        fr = hardwareMap.get(DcMotor.class, "frontright");
        bl = hardwareMap.get(DcMotor.class, "backleft");
        br = hardwareMap.get(DcMotor.class, "backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        intakefront = hardwareMap.get(DcMotor.class, "intakefront");
        intakeback = hardwareMap.get(DcMotor.class, "intakeback");
        shootr = hardwareMap.get(DcMotor.class, "shootr");
        shootl = hardwareMap.get(DcMotor.class, "shootl");

        reargate = hardwareMap.get(Servo.class, "reargate");
        launchgate = hardwareMap.get(Servo.class, "launchgate");

        launchgate.setPosition(0.5);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br, intakefront, intakeback, shootr, shootl}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Blue Alliance Teleop Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.left_stick_x;

            double scale = 1;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double flPower = (y + x + rx) / denominator * scale;
            double blPower = (y - x + rx) / denominator * scale;
            double frPower = (y - x - rx) / denominator * scale;
            double brPower = (y + x - rx) / denominator * scale;

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            if (gamepad1.right_bumper) {
                intakefront.setPower(-1.0);
            } else {
                intakefront.setPower(0.0);
            }

            if (gamepad1.left_bumper) {
                intakeback.setPower(-1.0);
            } else {
                intakeback.setPower(0.0);
            }

            if (gamepad1.right_trigger > 0.1) {
                shootr.setPower(1.0);
                shootl.setPower(-1.0);
            } else {
                shootr.setPower(0.0);
                shootl.setPower(0.0);
            }

            if (gamepad1.left_trigger > 0.1) {
                launchgate.setPosition(0.8);
                sleep(200);
            } else {
                launchgate.setPosition(0.5);
            }

            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("FL", flPower);
            telemetry.addData("FR", frPower);
            telemetry.addData("BL", blPower);
            telemetry.addData("BR", brPower);
            telemetry.addData("Intake Front", intakefront.getPower());
            telemetry.addData("Intake Back", intakeback.getPower());
            telemetry.addData("Shooter R", shootr.getPower());
            telemetry.addData("Shooter L", shootl.getPower());
            telemetry.addData("Launchgate", launchgate.getPosition());
            telemetry.update();
        }
    }
}

