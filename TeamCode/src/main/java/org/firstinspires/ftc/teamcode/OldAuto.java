package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Old Auto", group = "Unused")
public class OldAuto extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "blueObject_v1.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blueObject"
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private float elementPos;
    private double turnAmount;
    Recognition recognition = null;

    LiftSubsystem lift;
    StowSubsystem stow;
    Down stowDown;

    @Override
    public void runOpMode() {
        stowDown = new Down(stow);

        CommandScheduler scheduler = CommandScheduler.getInstance();

        initTfod();
        tfod.setZoom(1.3);

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(12, -63.339, Math.toRadians(90));

        driveTrain.setPoseEstimate(startPose);
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PoseStorage.currentPose = driveTrain.getPoseEstimate();

        TrajectorySequence traj1 = driveTrain.trajectorySequenceBuilder(startPose)
                .forward(26)
                .build();

        TrajectorySequence traj2 = driveTrain.trajectorySequenceBuilder(traj1.end())
                .waitSeconds(0.25)
                .back(15)
                .splineToSplineHeading(new Pose2d(46, -35, Math.toRadians(180)), Math.toRadians(20))
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(14, -58.5), Math.toRadians(180))
                .forward(35)
                .splineToSplineHeading(new Pose2d(-55, -45, Math.toRadians(135)), Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size() != 0) {
            recognition = currentRecognitions.get(0);
            elementPos = recognition.getRight() + recognition.getLeft() / 2;
            if (elementPos < 300) turnAmount = 45.0;
            else if (elementPos >= 300) turnAmount = 0.0;
            else turnAmount = -45.0;
        } else turnAmount = -45.0;


        telemetry.addData("Recognition", recognition);
        telemetry.addData("Pos", elementPos);

        driveTrain.followTrajectorySequence(traj1);
        scheduler.schedule(stowDown);
        sleep(1000);
        stow.stow();
        driveTrain.followTrajectorySequence(traj2);

        visionPortal.close();
        PoseStorage.hasAutoRun = true;
        PoseStorage.currentPose = driveTrain.getPoseEstimate();
    }

    private void runCommand(CommandBase command) {
        command.initialize();
        while (!command.isFinished()) {
            if (command.isFinished()) break;
        }
    }

    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);
        visionPortal = builder.build();

        tfod.setMinResultConfidence(0.85f);
    }

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
}
