package org.firstinspires.ftc.teamcode.Arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static final double ROBOT_WIDTH = 420.0; //mm
    public static final double ROBOT_LENGTH = 440.0; //mm

    public static final int LIFT_VELOCITY = 3000;
    public static final int LIFT_VEL_TO_STOW = 3100;
    public static final int SHOULDER_VELOCITY = 2300;

    public static final int LIFT_LIMIT_TOP = -5000;
    public static final int LIFT_LIMIT_BOTTOM = 0;

//    Rest/collect pos
    public static final int SHOULDER_POS_REST = 0;
    public static final int LIFT_POS_REST = 0;
    public static double STOW_POS_REST = 1.0;

//    Low score pos
    public static final int SHOULDER_POS_LOW = 431;
    public static final int LIFT_POS_LOW = -580;
    public static double STOW_POS_LOW = 0.5;

//    Mid score pos
//    public static final int SHOULDER_POS_MID = 2093;
    public static final int SHOULDER_POS_MID = 635;
//    public static final int LIFT_POS_MID = -2684;
    public static final int LIFT_POS_MID = -2022;
    public static double STOW_POS_MID = 0.55;

//    Climb pos
    public static final int SHOULDER_POS_CLIMB = 1450;
    public static final int LIFT_POS_CLIMB = -4060;
    public static double STOW_POS_CLIMB = 0.75;

//    Dump pos
    public static final int SHOULDER_POS_DUMP = 523;
    public static final int LIFT_POS_DUMP = -2170;
    public static double STOW_POS_DUMP = 0.5;

    public static double STOW_POS_STOW = 0.0;
}
