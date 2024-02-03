package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Arm.ArmCommand;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.Manip.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawOne;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawTwo;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.Stow;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.RRCommands.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.RRCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Vision.VisionUpdatePose;

@Autonomous(name = "Red Wing Side Cycle", group = "Red Autos")
public class AutoRedWingCycle extends CommandOpMode {
    private static final String TFOD_MODEL_ASSET = "redObject_v2.tflite";
    private static final String[] LABELS = {
            "redObject"
    };

    private float elementPos;
    private double turnAmount;
    private double fwd;
    private double aprilTagY;

    VisionSubsystem vision;
    VisionUpdatePose visionUpdatePose;
    MecanumDrive driveTrain;
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
        driveTrain = new MecanumDrive(hardwareMap, telemetry, false);
        vision = new VisionSubsystem(hardwareMap, "Webcam 1", TFOD_MODEL_ASSET, LABELS, telemetry);
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
        visionUpdatePose = new VisionUpdatePose(vision, driveTrain);

        vision.enableTfod();
        vision.disableAprilTag();

        claw.clawOneToPos(0);
        claw.clawTwoToPos(1);
        sleep(200);
        stow.stow();

        Pose2d startPose = new Pose2d(-35, -63.339, Math.toRadians(90));
        driveTrain.setPoseEstimate(startPose);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT);

        PoseStorage.pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT;
        PoseStorage.currentPose = driveTrain.getPoseEstimate();
        PoseStorage.hasAutoRun = false;

        FtcDashboard.getInstance().startCameraStream(vision.stream, 15);

        waitForStart();
        if (isStopRequested()) return;

        Recognition recognition = vision.getTfodDetection();
        if (recognition != null) {
            elementPos = recognition.getRight() + recognition.getLeft() / 2;
            if (elementPos < 275) {
                // Left
                turnAmount = 55.0;
                fwd = 20;
                aprilTagY = -21.5;
            }
            else if (elementPos >= 275) {
                // Middle
                turnAmount = -20.0;
                fwd = 25;
                aprilTagY = -30.0; //-28.75
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

        vision.disableTfod();
//        vision.enableAprilTag();

        TrajectorySequence traj1 = driveTrain.trajectorySequenceBuilder(startPose)
                .forward(fwd)
                .build();

        TrajectorySequence traj2 = driveTrain.trajectorySequenceBuilder(traj1.end())
                .back(7)
                .splineToConstantHeading(new Vector2d(-56, -59), Math.toRadians(90))
                .forward(7)
                .build();

        register(vision);
        vision.setDefaultCommand(visionUpdatePose);

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
                        new TrajectoryRunner(driveTrain, traj2),
                        new TurnCommand(driveTrain, Math.toRadians(20)),
//                        new ArmCommand(
//
//                        ),

                        new SequentialCommandGroup(
                                new InstantCommand(() -> PoseStorage.currentPose = driveTrain.getPoseEstimate()),
                                new InstantCommand(() -> PoseStorage.hasAutoRun = true),
                                new InstantCommand(() -> PoseStorage.dumpingTag = 4),
                                new InstantCommand(() -> PoseStorage.pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT),
                                new InstantCommand(() -> telemetry.addData("PoseStorage saved", PoseStorage.hasAutoRun)),
                                new InstantCommand(() -> telemetry.addData("Pose", PoseStorage.currentPose)),
                                new InstantCommand(() -> telemetry.update())
                        )
                )
        );
    }
}
