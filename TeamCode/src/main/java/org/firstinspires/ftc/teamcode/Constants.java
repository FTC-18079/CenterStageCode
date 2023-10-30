package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double ROBOT_WIDTH = 420.0; //mm
    public static final double ROBOT_LENGTH = 440.0; //mm

    public static final String[] telemetryData = {
            "Lift Encoder",
            "Shoulder Encoder",
            "Wrist Pos",
            "Claw One Pos",
            "Claw Two Pos"
    };

    public static final int LIFT_VELOCITY = 2350;
    public static final int SHOULDER_VELOCITY = 2400;

    public static final int LIFT_LIMIT_TOP = -5750;
    public static final int LIFT_LIMIT_BOTTOM = 0;

//    Rest/collect pos
    public static final int SHOULDER_POS_REST = 0;
    public static final int LIFT_POS_REST = 0;
    public static final double STOW_POS_REST = 0.678;

//    Low score pos
    public static final int SHOULDER_POS_LOW = 2100;
    public static final int LIFT_POS_LOW = -1000;
    public static final double STOW_POS_LOW = 1.0;

//    Mid score pos
    public static final int SHOULDER_POS_MID = 1229;
    public static final int LIFT_POS_MID = 0;
    public static final double STOW_POS_MID = 1.0;

//    High score pos
    public static final int SHOULDER_POS_HIGH = 1229;
    public static final int LIFT_POS_HIGH = -1650;
    public static final double STOW_POS_HIGH = 1.0;
}
