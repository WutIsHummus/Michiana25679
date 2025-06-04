package org.firstinspires.ftc.teamcode.helpers.data;


public class Enums {
    public enum Intake {
        Extended,
        Transfer,
        Spin
    }
    public enum OutTake {
        Transfer,
        Deposit,
        Specimen,
        wall
    }

    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        BLACK,
        WHITE,
        GRAY,
        UNKNOWN
    }

    public enum IntakeState {
        SEARCHING,
        REJECTING
    }

}


