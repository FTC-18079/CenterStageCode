package org.firstinspires.ftc.teamcode.Vision;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util.Field2d;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

public class ATVision {
    private final WebcamName camera;
    private final Field2d fieldPose;
    private final VisionConstants.Camera cameraSettings;
    private final MecanumDrive chassis;
    private int updates = 0;
    private int failedUpdates = 0;
    private double lastUpdateTime = 0;
    private int aprilTagCount = 0;

    public ATVision(VisionConstants.Camera cameraSettings, Field2d fieldPose, MecanumDrive chassis, HardwareMap hMap) {
        this.cameraSettings = cameraSettings;
        this.fieldPose = fieldPose;
        this.chassis = chassis;

        camera = hMap.get(WebcamName.class, cameraSettings.name);
    }

//    private AprilTagDetection processSingleTarget(Pose2d robotPose, )


}
