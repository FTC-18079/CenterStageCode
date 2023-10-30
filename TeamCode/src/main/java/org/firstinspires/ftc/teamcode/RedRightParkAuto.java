package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftToPos;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderToPos;
import org.firstinspires.ftc.teamcode.Manip.Claw.AutoMoveClaw;
import org.firstinspires.ftc.teamcode.Manip.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.Stow;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowToPos;
import org.firstinspires.ftc.teamcode.Manip.Wrist.WristCommand;
import org.firstinspires.ftc.teamcode.Manip.Wrist.WristSubsystem;
import org.firstinspires.ftc.teamcode.RRCommands.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.RRCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Red Right Park", group = "Autos")
public class RedRightParkAuto extends CommandOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "redObject_v1.tflite";
    private static final String[] LABELS = {
            "redObject"
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private float elementPos;
    private double turnAmount;
    Recognition recognition = null;

    StowSubsystem stow;
    ClawSubsystem claw;
    WristSubsystem wrist;
    ShoulderSubsystem shoulder;
    LiftSubsystem lift;
    Stow stowUp;
    Down stowDown;
    AutoMoveClaw moveClaw;
    WristCommand moveWrist;

    @Override
    public void initialize() {
        // Subsystems
        stow = new StowSubsystem(hardwareMap, "stow");
        claw = new ClawSubsystem(hardwareMap, "clawOne", "clawTwo");
        wrist = new WristSubsystem(hardwareMap, "wrist");
        shoulder = new ShoulderSubsystem(hardwareMap, "shoulder1", "shoulder2", "shoulderTouch", telemetry);
        lift = new LiftSubsystem(hardwareMap, "lift", "liftTouch", telemetry);

        // Commands
        stowUp = new Stow(stow);
        stowDown = new Down(stow);
        moveClaw = new AutoMoveClaw(claw, wrist, shoulder);
        moveWrist = new WristCommand(wrist);

        initTfod();
        tfod.setZoom(1.15);

        claw.clawOneToPos(0);
        claw.clawTwoToPos(0);
        stow.stow();
        wrist.toPos(0);

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(12, -63.339, Math.toRadians(90));

        driveTrain.setPoseEstimate(startPose);
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PoseStorage.currentPose = driveTrain.getPoseEstimate();

        TrajectorySequence traj1 = driveTrain.trajectorySequenceBuilder(startPose)
                .forward(22)
                .build();

        TrajectorySequence traj2 = driveTrain.trajectorySequenceBuilder(traj1.end())
                .back(9)
                .splineToSplineHeading(new Pose2d(46, -35, Math.toRadians(0)), Math.toRadians(20))
                .build();

        TrajectorySequence traj3 = driveTrain.trajectorySequenceBuilder(traj2.end())
                .waitSeconds(0.5)
                .back(0.5)
                .splineToConstantHeading(new Vector2d(60, -61), Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size() != 0) {
            recognition = currentRecognitions.get(0);
            elementPos = recognition.getRight() + recognition.getLeft() / 2;
            if (elementPos < 275) turnAmount = 50.0;
            else if (elementPos >= 275) turnAmount = 35.0;
            else turnAmount = -55.0;
        } else turnAmount = -55.0;

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new TrajectoryRunner(driveTrain, traj1), //Follow trajectory 1
                        new TurnCommand(driveTrain, Math.toRadians(turnAmount)), //Turn to face game element's spike mark
                        new InstantCommand(stowDown), //Bring down stow
                        new WaitCommand(750), //Wait .75s
                        new InstantCommand(moveClaw), //Open claw to score spike mark
                        new WaitCommand(750), //Wait 1s
                        new InstantCommand(stowUp), //Bring stow up
                        new WaitCommand(750), //Wait 1s
                        new TurnCommand(driveTrain, Math.toRadians(turnAmount * -1)),
                        new ParallelCommandGroup(
                                new ShoulderToPos(shoulder, () -> 460, () -> Constants.SHOULDER_VELOCITY, telemetry),
                                new InstantCommand(moveWrist),
                                new TrajectoryRunner(driveTrain, traj2),
                                new LiftToPos(lift, () -> -2100, () -> Constants.LIFT_VELOCITY, telemetry),
                                new StowToPos(stow, () -> 0.5)
                        ), //Drive to backboard while brining arm up to score
                        new WaitCommand(2000), //Wait 2s
                        new InstantCommand(moveClaw), //Open claw to score on backboard
                        new WaitCommand(1000), //Wait 2s
                        new ParallelCommandGroup(
                                new ShoulderToPos(shoulder, () -> 0, () -> Constants.SHOULDER_VELOCITY, telemetry),
                                new LiftToPos(lift, () -> 0, () -> Constants.LIFT_VELOCITY, telemetry),
                                new InstantCommand(stowUp)
                        ),
                        new TrajectoryRunner(driveTrain, traj3)
                )
        );
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
