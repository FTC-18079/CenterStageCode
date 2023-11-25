package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double ROBOT_WIDTH = 420.0; //mm
    public static final double ROBOT_LENGTH = 440.0; //mm

    public static final int LIFT_VELOCITY = 2450;
    public static final int LIFT_VEL_TO_STOW = 2600;
    public static final int SHOULDER_VELOCITY = 2300;

    public static final int LIFT_LIMIT_TOP = -5750;
    public static final int LIFT_LIMIT_BOTTOM = 0;

//    Rest/collect pos
    public static final int SHOULDER_POS_REST = 0;
    public static final int LIFT_POS_REST = 0;
    public static final double STOW_POS_REST = 0.7;

//    Low score pos
    public static final int SHOULDER_POS_LOW = 431;
    public static final int LIFT_POS_LOW = -580;
    public static final double STOW_POS_LOW = 0.15;

//    Mid score pos
//    public static final int SHOULDER_POS_MID = 2093;
    public static final int SHOULDER_POS_MID = 635;
//    public static final int LIFT_POS_MID = -2684;
    public static final int LIFT_POS_MID = -2022;
    public static final double STOW_POS_MID = 0.1;

//    Climb pos
    public static final int SHOULDER_POS_CLIMB = 1450;
    public static final int LIFT_POS_CLIMB = -4060;
    public static final double STOW_POS_CLIMB = 0.25;
}
