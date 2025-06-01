package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo Test", group = "TeleOp")
public class ServoTesting extends OpMode {

    // All your servos
    Servo ptor, ptol, led, lede, sweeper;
    Servo intakepivot;
    Servo outtakeclaw, outtakearmr, outakearml, outakelinkage;
    Servo intakearmr, intakearml;
    Servo hangr, hangl;

    // Dashboard-tunable targets
    public static double ptorTarget         = 0.6;
    public static double ptolTarget         = 0.44;
    public static double ledTarget          = 0.5;
    public static double ledeTarget         = 0.5;
    public static double sweeperTarget      = 0.67;
    public static double intakepivotTarget  = 0.28;
    public static double outtakeclawTarget  = 0.3;
    public static double outtakearmrTarget  = 1;
    public static double outakearmlTarget   = 1;
    public static double outakelinkageTarget= 0.57;
    public static double intakearmrTarget   = 0.2;
    public static double intakearmlTarget   = 0.2;
    public static double hangrTarget        = 0.5;
    public static double hanglTarget        = 0.5;

    @Override
    public void init() {
        // Map each servo by the exact name you gave
        ptor          = hardwareMap.get(Servo.class, "ptor");
        ptol          = hardwareMap.get(Servo.class, "ptol");
        led           = hardwareMap.get(Servo.class, "led");
        lede          = hardwareMap.get(Servo.class, "lede");
        sweeper       = hardwareMap.get(Servo.class, "sweeper");
        intakepivot   = hardwareMap.get(Servo.class, "intakepivot");
        outtakeclaw   = hardwareMap.get(Servo.class, "outtakeclaw");
        outtakearmr   = hardwareMap.get(Servo.class, "outaker");
        outakearml    = hardwareMap.get(Servo.class, "outakel");
        outakelinkage = hardwareMap.get(Servo.class, "outakelinkage");
        intakearmr    = hardwareMap.get(Servo.class, "intaker");
        intakearml    = hardwareMap.get(Servo.class, "intakel");
        hangr         = hardwareMap.get(Servo.class, "hangr");
        hangl         = hardwareMap.get(Servo.class, "hangl");

        outtakearmr.setDirection(Servo.Direction.REVERSE);
        intakearmr.setDirection(Servo.Direction.REVERSE);
        hangr.setDirection(Servo.Direction.REVERSE);

        // Combine telemetry with dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Push positions to each servo
        ptor.setPosition(ptorTarget);
        ptol.setPosition(ptolTarget);
        led.setPosition(ledTarget);
        lede.setPosition(ledeTarget);
        sweeper.setPosition(sweeperTarget);
        intakepivot.setPosition(intakepivotTarget);
        outtakeclaw.setPosition(outtakeclawTarget);
        outtakearmr.setPosition(outtakearmrTarget);
        outakearml.setPosition(outakearmlTarget);
        outakelinkage.setPosition(outakelinkageTarget);
        intakearmr.setPosition(intakearmrTarget);
        intakearml.setPosition(intakearmlTarget);
        hangr.setPosition(hangrTarget);
        hangl.setPosition(hanglTarget);

        // Telemetry for tuning
        telemetry.addData("ptor",         ptorTarget);
        telemetry.addData("ptol",         ptolTarget);
        telemetry.addData("led",          ledTarget);
        telemetry.addData("lede",         ledeTarget);
        telemetry.addData("sweeper",      sweeperTarget);
        telemetry.addData("intakepivot",  intakepivotTarget);
        telemetry.addData("outtakeclaw",  outtakeclawTarget);
        telemetry.addData("outtakearmr",  outtakearmrTarget);
        telemetry.addData("outakearml",   outakearmlTarget);
        telemetry.addData("outakelinkage",outakelinkageTarget);
        telemetry.addData("intakearmr",   intakearmrTarget);
        telemetry.addData("intakearml",   intakearmlTarget);
        telemetry.addData("hangr",        hangrTarget);
        telemetry.addData("hangl",        hanglTarget);
        telemetry.update();
    }
}
