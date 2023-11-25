package org.firstinspires.ftc.teamcode.Roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class PoseStorage {
    public static boolean hasAutoRun = false;
    public static Pose2d currentPose = new Pose2d();
    public static RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
}
