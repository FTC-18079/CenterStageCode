package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class NewAuto extends CommandOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "blueObject_v1.tflite";
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

    @Override
    public void initialize() {
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

        register(stow);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> new TrajectoryRunner(driveTrain, traj1).initialize()),
                        new InstantCommand(() -> stow.down()),
                        new WaitCommand(1000),
                        new InstantCommand(() -> new TrajectoryRunner(driveTrain, traj2).initialize())
                )
        );
//        CommandScheduler.getInstance().schedule(new TrajectoryRunner(driveTrain, traj1));
//        CommandScheduler.getInstance().schedule(new Down(stow));
//        sleep(1000);
//        CommandScheduler.getInstance().schedule(new TrajectoryRunner(driveTrain, traj2));
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
}
