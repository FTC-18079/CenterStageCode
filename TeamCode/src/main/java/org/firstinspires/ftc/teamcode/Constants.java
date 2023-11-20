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
    public static final int LIFT_VEL_TO_STOW = 2450;
    public static final int SHOULDER_VELOCITY = 2400;

    public static final int LIFT_LIMIT_TOP = -5750;
    public static final int LIFT_LIMIT_BOTTOM = 0;

    //    Rest/collect pos
    public static final int SHOULDER_POS_REST = 0;
    public static final int LIFT_POS_REST = 0;
    public static final double STOW_POS_REST = 0.7;

    //    Low score pos
    public static final int SHOULDER_POS_LOW = 635;
    public static final int LIFT_POS_LOW = -2022;
    public static final double STOW_POS_LOW = 0.1;

    //    Mid score pos
//    public static final int SHOULDER_POS_MID = 2093;
    public static final int SHOULDER_POS_MID = 431;
    //    public static final int LIFT_POS_MID = -2684;
    public static final int LIFT_POS_MID = -580;
    public static final double STOW_POS_MID = 0.15;

    //    Climb pos
    public static final int SHOULDER_POS_CLIMB = 1560;
    public static final int LIFT_POS_CLIMB = -4060;
    public static final double STOW_POS_CLIMB = 1.0;
}