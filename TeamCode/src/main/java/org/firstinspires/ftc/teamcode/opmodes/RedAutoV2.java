package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Arm.ArmCommand;
import org.firstinspires.ftc.teamcode.Arm.ArmConstants;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftToPos;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderToPos;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.Manip.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawOne;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawTwo;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.Stow;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowToPos;
import org.firstinspires.ftc.teamcode.RRCommands.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.RRCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Vision.VisionUpdatePose;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Disabled
@Autonomous(name = "Red Auto V2", group = "Tests")
public class RedAutoV2 extends CommandOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "redObject_v2.tflite";
    private static final String[] LABELS = {
            "redObject"
    };
    private float elementPos;
    private double turnAmount;
    private double fwd;
    VisionSubsystem visionSubsystem;
    VisionUpdatePose visionUpdatePose;
    MecanumDrive driveTrain;
//    SampleMecanumDrive driveTrain;
    StowSubsystem stow;
    ClawSubsystem claw;
    ShoulderSubsystem shoulder;
    LiftSubsystem lift;
    Stow stowUp;
    Down stowDown;
    MoveClawOne moveClawOne;
    MoveClawTwo moveClawTwo;
    private RevBlinkinLedDriver led;
    private TrajectorySequence traj1, traj2, traj3;
    private double aprilTagY;

    @Override
    public void initialize() {
        // Subsystems
        driveTrain = new MecanumDrive(hardwareMap, telemetry, false);
//        driveTrain = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(12, -63.339, Math.toRadians(90));

        visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1", TFOD_MODEL_ASSET, LABELS, telemetry);
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
        visionUpdatePose = new VisionUpdatePose(visionSubsystem, driveTrain);

        visionSubsystem.enableTfod();
        visionSubsystem.disableAprilTag();

        claw.clawOneToPos(0);
        claw.clawTwoToPos(1);
        sleep(200);
        stow.stow();

        driveTrain.setPoseEstimate(startPose);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT);

        PoseStorage.pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT;
        PoseStorage.currentPose = driveTrain.getPoseEstimate();
        PoseStorage.hasAutoRun = false;

        FtcDashboard.getInstance().startCameraStream(visionSubsystem.stream, 30);

        waitForStart();
        if (isStopRequested()) return;

        Recognition recognition = visionSubsystem.getTfodDetection();
        if (recognition != null) {
            elementPos = recognition.getLeft() + recognition.getRight() / 2;
            if (elementPos < 275) {
                // Left
                turnAmount = 62.0;
                fwd = 20;
                aprilTagY = -20.5;
            }
            else if (elementPos >= 275) {
                // Middle
                turnAmount = -15.0;
                fwd = 26;
                aprilTagY = -28.75;
            }
            else {
                // Right
                turnAmount = -50.0;
                fwd = 20;
                aprilTagY = -38.0;
            }
        } else {
            // Right
            turnAmount = -50.0;
            fwd = 20;
            aprilTagY = -38.0;
        }

        visionSubsystem.disableTfod();
        visionSubsystem.enableAprilTag();

        TrajectorySequence traj1 = driveTrain.trajectorySequenceBuilder(startPose)
                .forward(fwd)
                .build();

        TrajectorySequence traj2 = driveTrain.trajectorySequenceBuilder(traj1.end())
                .back(fwd - 13)
                .splineToSplineHeading(new Pose2d(50.2, aprilTagY, Math.toRadians(0)), Math.toRadians(20))
                .build();

        TrajectorySequence traj3 = driveTrain.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(45, -58, Math.toRadians(0)))
                .forward(9)
                .build();

        register(visionSubsystem);
        visionSubsystem.setDefaultCommand(visionUpdatePose);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new TrajectoryRunner(driveTrain, traj1), // Follow trajectory 1
                        new TurnCommand(driveTrain, Math.toRadians(turnAmount)), // Turn to face game element's spike mark
                        new InstantCommand(stowDown), // Bring down stow
                        new WaitCommand(500), // Wait .5s
                        new InstantCommand(moveClawOne), // Open claw to score spike mark
                        new WaitCommand(500), // Wait .5s
                        new InstantCommand(stowUp), // Bring stow up
                        new WaitCommand(500), // Wait .5s
                        new TurnCommand(driveTrain, Math.toRadians(turnAmount * -1)),
                        /*new ArmCommand(
                                shoulder,
                                lift,
                                stow,
                                () -> ArmConstants.SHOULDER_POS_LOW,
                                () -> ArmConstants.LIFT_POS_LOW,
                                () -> ArmConstants.STOW_POS_LOW,
                                telemetry
                        ),*/
                        new TrajectoryRunner(driveTrain, traj2), // Drive to backboard while brining arm up to score
                        new InstantCommand(moveClawTwo), // Open claw to score on backboard

                        // TODO: SUPER SKETCHY, this would be replaced for updating estimate by using apriltags
//                        new InstantCommand(() -> driveTrain.setPoseEstimate(new Pose2d(50, aprilTagY, Math.toRadians(12)))),

                        new WaitCommand(600), // Wait 0.6s
                        /*new ArmCommand(
                                shoulder,
                                lift,
                                stow,
                                () -> ArmConstants.SHOULDER_POS_REST,
                                () -> ArmConstants.LIFT_POS_REST,
                                () -> ArmConstants.STOW_POS_REST,
                                telemetry
                        ),*/
                        new ParallelRaceGroup(
                                new TrajectoryRunner(driveTrain, traj3),
                                new WaitCommand(6000)
                        ),
                        new InstantCommand(() -> shoulder.stopVelocity()),
                        new InstantCommand(() -> driveTrain.drive(new Pose2d())),

                        // Update pose storage and telemetry
                        new SequentialCommandGroup(
                                new InstantCommand(() -> PoseStorage.currentPose = new Pose2d(
                                        driveTrain.getPoseEstimate().vec(),
                                        driveTrain.getPoseEstimate().getHeading() + Math.toRadians(6)
                                )),
                                new InstantCommand(() -> PoseStorage.hasAutoRun = true),
                                new InstantCommand(() -> PoseStorage.pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT),
                                new InstantCommand(() -> telemetry.addData("PoseStorage saved", PoseStorage.hasAutoRun)),
                                new InstantCommand(() -> telemetry.addData("Pose", PoseStorage.currentPose)),
                                new InstantCommand(() -> telemetry.update())
                        )
                )
        );
    }
}