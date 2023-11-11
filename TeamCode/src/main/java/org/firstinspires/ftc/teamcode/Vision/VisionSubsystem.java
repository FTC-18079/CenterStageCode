package org.firstinspires.ftc.teamcode.Vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class VisionSubsystem extends SubsystemBase {
    private final WebcamName camera;
    private final Telemetry tele;

    private final AprilTagProcessor aprilTagProcessor;
    private final TfodProcessor tfodProcessor;
    private final VisionPortal visionPortal;

    public VisionSubsystem(HardwareMap hMap, String name, Telemetry tele) {
        camera = hMap.get(WebcamName.class, name);
        this.tele = tele;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        tfodProcessor = new TfodProcessor.Builder()
                .setModelAssetName("redObject_v1.tflite")
                .setModelLabels(new String[] {"redObject"})
                .setModelAspectRatio(16.0 / 9.0)
                .build();
        tfodProcessor.setMinResultConfidence(0.85f);

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
}
