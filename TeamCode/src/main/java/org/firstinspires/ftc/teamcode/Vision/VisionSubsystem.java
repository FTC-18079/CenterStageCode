package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    public final CameraStreamProcessor stream = new CameraStreamProcessor();

    public VisionSubsystem(HardwareMap hMap, String name, String tfodAsset, String[] tfodLabels, Telemetry tele) {
        camera = hMap.get(WebcamName.class, name);
        this.tfodAsset = tfodAsset;
        this.tfodLabels = tfodLabels;
        this.tele = tele;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        aprilTagProcessor.setDecimation(3);

        tfodProcessor = new TfodProcessor.Builder()
                .setModelAssetName(this.tfodAsset)
                .setModelLabels(this.tfodLabels)
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
                .addProcessor(stream)
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

    // TODO: Find a fix for why updating pose from big tags behaves strangely
    public void updatePoseAprilTag(int targetTagId, MecanumDrive drive) {
        AprilTagDetection aprilTag = getAprilTagDetection(targetTagId);
        tele.addData("Target tag", targetTagId);
        tele.addData("Tag found", aprilTag != null);

        if (aprilTag != null) {
            Pose2d tagPose = getTagPose(targetTagId);

            double range = aprilTag.ftcPose.range;
            double angle = aprilTag.ftcPose.yaw;

            double xTranslation = (range * Math.cos(angle) + 6.0);
            double yTranslation = 4.33 * (tagPose.getX() < 0 ? 1 : -1);
            double angleRot = tagPose.getX() < 0 ? 180 : 0;

            double x = xTranslation * ((tagPose.getX() < 0) ? 1 : -1) + tagPose.getX();
            double y = (range * Math.sin(angle) + yTranslation) + tagPose.getY();

            Pose2d newRobotPose = new Pose2d(x, y, -angle + Math.toRadians(angleRot));

            if (range <= 36 && Math.abs(angle) < 40.0 && updateTelemetry(drive.getPoseEstimate())) {
                drive.setPoseEstimate(newRobotPose);
            }
        }
    }

    public boolean updateTelemetry(Pose2d pose) {
        double heading = Math.toRadians(pose.getHeading());
        if (Math.abs(heading - 180) <= 30) {
            return true;
        } else if (heading <= 35) {
            return true;
        } else if (Math.abs(heading - 360) <= 35) {
            return true;
        } else return false;
    }

    public Pose2d getTagPose(int targetTag) {
        Pose2d pose = new Pose2d();
        if (targetTag == 2) pose = new Pose2d(60, 35);
        else if(targetTag == 5) pose = new Pose2d(60, -35);
        else if(targetTag == 7) pose = new Pose2d(-70, -40.75);
        else if(targetTag == 10) pose = new Pose2d(-70, 40.75);

        return pose;
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
