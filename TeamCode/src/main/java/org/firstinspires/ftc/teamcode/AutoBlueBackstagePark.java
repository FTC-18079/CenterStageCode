package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Arm.ArmCommand;
import org.firstinspires.ftc.teamcode.Arm.ArmConstants;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawOne;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawTwo;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.Stow;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.RRCommands.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.RRCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Blue Backstage - Park", group = "Blue Autos")
public class AutoBlueBackstagePark extends CommandOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "blueObject_v1.tflite";
    private static final String[] LABELS = {
            "blueObject"
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private float elementPos;
    private double turnAmount;
    private double fwd;
    private double aprilTagY;
    Recognition recognition = null;

    StowSubsystem stow;
    ClawSubsystem claw;
    ShoulderSubsystem shoulder;
    LiftSubsystem lift;
    Stow stowUp;
    Down stowDown;
    MoveClawOne moveClawOne;
    MoveClawTwo moveClawTwo;

    private RevBlinkinLedDriver led;

    @Override
    public void initialize() {
        // Subsystems
        stow = new StowSubsystem(hardwareMap, "stow");
        claw = new ClawSubsystem(hardwareMap, "clawOne", "clawTwo");
        shoulder = new ShoulderSubsystem(hardwareMap, "shoulder1", "shoulder2", "shoulderTouch", telemetry);
        lift = new LiftSubsystem(hardwareMap, "lift", "liftTouch", telemetry);

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        // Commands
        stowUp = new Stow(stow);
        stowDown = new Down(stow);
        moveClawOne = new MoveClawOne(claw);
        moveClawTwo = new MoveClawTwo(claw);

        initTfod();
        tfod.setZoom(1.0);
        tfod.setClippingMargins(0, 100, 130, 0);

        claw.clawOneToPos(0);
        claw.clawTwoToPos(1);
        sleep(200);
        stow.stow();

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(12, 63.339, Math.toRadians(-90));

        driveTrain.setPoseEstimate(startPose);
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);

        PoseStorage.pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT;
        PoseStorage.currentPose = driveTrain.getPoseEstimate();
        PoseStorage.hasAutoRun = false;

        waitForStart();
        if (isStopRequested()) return;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size() != 0) {
            recognition = currentRecognitions.get(0);
            elementPos = recognition.getRight() + recognition.getLeft() / 2;
            if (elementPos < 275) {
                // Left
                turnAmount = 60.0;
                fwd = 20;
                aprilTagY = 44.0;
            }
            else if (elementPos >= 275) {
                // Middle
                turnAmount = 15.0;
                fwd = 26;
                aprilTagY = 38.5;
            }
            else {
                // Right
                turnAmount = -60.0;
                fwd = 20;
                aprilTagY = 30.0;
            }
        } else {
            // Right
            turnAmount = -60.0;
            fwd = 20;
            aprilTagY = 30.0;
        }

        TrajectorySequence traj1 = driveTrain.trajectorySequenceBuilder(startPose)
                .forward(fwd)
                .build();

        TrajectorySequence traj2 = driveTrain.trajectorySequenceBuilder(traj1.end())
                .back(fwd - 13)
                .splineToSplineHeading(new Pose2d(50.5, aprilTagY, Math.toRadians(0)), Math.toRadians(-20))
                .build();

        TrajectorySequence traj3 = driveTrain.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(45, 58, Math.toRadians(0)))
                .forward(9)
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new TrajectoryRunner(driveTrain, traj1), //Follow trajectory 1
                        new TurnCommand(driveTrain, Math.toRadians(turnAmount)), //Turn to face game element's spike mark
                        new InstantCommand(stowDown), //Bring down stow
                        new WaitCommand(500), //Wait .5s
                        new InstantCommand(moveClawOne), //Open claw to score spike mark
                        new WaitCommand(500), //Wait .5s
                        new InstantCommand(stowUp), //Bring stow up
                        new WaitCommand(500), //Wait .5s
                        new TurnCommand(driveTrain, Math.toRadians(turnAmount * -1)),
                        new ArmCommand(
                                shoulder,
                                lift,
                                stow,
                                () -> ArmConstants.SHOULDER_POS_LOW,
                                () -> ArmConstants.LIFT_POS_LOW,
                                () -> ArmConstants.STOW_POS_LOW,
                                telemetry
                        ),
                        new TrajectoryRunner(driveTrain, traj2), // Drive to backboard while brining arm up to score
                        new InstantCommand(moveClawTwo), //Open claw to score on backboard

                        // TODO: SUPER SKETCHY, this would be replaced for updating estimate by using apriltags
//                        new InstantCommand(() -> driveTrain.setPoseEstimate(new Pose2d(50, aprilTagY, Math.toRadians(-12)))),

                        new WaitCommand(600), // Wait .6s
                        new ArmCommand(
                                shoulder,
                                lift,
                                stow,
                                () -> ArmConstants.SHOULDER_POS_REST,
                                () -> ArmConstants.LIFT_POS_REST,
                                () -> ArmConstants.STOW_POS_REST,
                                telemetry
                        ),
                        new ParallelRaceGroup(
                                new TrajectoryRunner(driveTrain, traj3),
                                new WaitCommand(6000)
                        ),
                        new InstantCommand(() -> shoulder.stopVelocity()),
                        new InstantCommand(() -> driveTrain.setWeightedDrivePower(new Pose2d())),

                        // Update pose storage and telemetry
                        new SequentialCommandGroup(
                                new InstantCommand(() -> PoseStorage.currentPose = driveTrain.getPoseEstimate()),
                                new InstantCommand(() -> PoseStorage.hasAutoRun = true),
                                new InstantCommand(() -> PoseStorage.pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT),
                                new InstantCommand(() -> telemetry.addData("PoseStorage saved", PoseStorage.hasAutoRun)),
                                new InstantCommand(() -> telemetry.addData("Pose", PoseStorage.currentPose)),
                                new InstantCommand(() -> telemetry.update())
                        )
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
