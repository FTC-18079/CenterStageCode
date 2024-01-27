package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    private final WebcamName camera;
    private final String tfodAsset;
    private final String[] tfodLabels;
    private final Telemetry tele;

    private final AprilTagProcessor aprilTagProcessor;
    private final TfodProcessor tfodProcessor;
    private final VisionPortal visionPortal;

    private Recognition tfodRecognition;
    private boolean targetTagFound = false;
    private boolean targetTfodFound = false;

    public VisionSubsystem(HardwareMap hMap, String name, String tFodAsset, String[] tfodLabels, Telemetry tele) {
        camera = hMap.get(WebcamName.class, name);
        this.tfodAsset = tFodAsset;
        this.tfodLabels = tfodLabels;
        this.tele = tele;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();
        aprilTagProcessor.setDecimation(3);

        tfodProcessor = new TfodProcessor.Builder()
                .setModelAssetName(tfodAsset)
                .setModelLabels(tfodLabels)
                .setModelAspectRatio(16.0 / 9.0)
                .build();
        tfodProcessor.setMinResultConfidence(0.85f);
        tfodProcessor.setClippingMargins(0, 100, 120, 0);
        tfodProcessor.setZoom(1.0);

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .enableLiveView(false)
                .setAutoStopLiveView(true)
                .addProcessor(aprilTagProcessor)
                .addProcessor(tfodProcessor)
                .build();

        disableAprilTag();
        disableTfod();
    }

    public AprilTagDetection getAprilTagDetection(int targetTagId) {
        targetTagFound = false;
        List<AprilTagDetection> detectionList;
        AprilTagDetection aprilTagDetection = null;

        detectionList = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : detectionList) {
            if (detection.metadata != null) { // This check for non-null metadata makes sure this tag is one we want to recognize
                if (detection.id == targetTagId) {
                    targetTagFound = true;
                    aprilTagDetection = detection;
                    break; // Ends loop if we find our AT
                }
            }
        }

        return aprilTagDetection;
    }

    public void updatePoseAprilTag(int targetTagId, MecanumDrive drive) {
        AprilTagDetection aprilTag = getAprilTagDetection(targetTagId);

        if (aprilTag != null) {
            double range = aprilTag.ftcPose.range;
            double angle = aprilTag.ftcPose.yaw;

            double x = -70 + (range * Math.cos(angle) + 6.0);
            double y = (targetTagId == 7 ? -36.5 : +36.5) + (range * Math.sin(angle) - 4.33);

            Pose2d newRobotPose = new Pose2d(x, y, -angle);

            if (range <= 48 && Math.abs(angle) < 40.0) {
                drive.setPoseEstimate(newRobotPose);
                drive.update();
            }
        }
    }

    public Recognition getTfodDetection() {
        targetTfodFound = false;
        tfodRecognition = null;
        List<Recognition> detectionList;

        detectionList = tfodProcessor.getRecognitions();
        if (detectionList.size() != 0) {
            targetTfodFound = true;
            tfodRecognition = detectionList.get(0);
        }

        return tfodRecognition;
    }

    public void enableAprilTag() {
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    public void disableAprilTag() {
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
    }

    public void enableTfod() {
        visionPortal.setProcessorEnabled(tfodProcessor, true);
    }

    public void disableTfod() {
        visionPortal.setProcessorEnabled(tfodProcessor, false);
    }

    public double getAprilTagAngle() {
        return getAprilTagDetection(5).ftcPose.bearing;
    }
}
