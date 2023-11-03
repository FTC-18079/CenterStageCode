package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RRCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name = "RedLeftAuto", group = "Autos")
public class RedLeftAuto extends LinearOpMode {
    LiftSubsystem lift;
    RunCommand runCommand;

    @Override
    public void runOpMode() {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap, telemetry);

        Pose2d startPose = new Pose2d(-35, -61.5, Math.toRadians(90));

        driveTrain.setPoseEstimate(startPose);
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PoseStorage.currentPose = driveTrain.getPoseEstimate();

        TrajectorySequence trajSeq = driveTrain.trajectorySequenceBuilder(startPose)
                .forward(26)
                .waitSeconds(0.25)
                .back(23)
                .strafeTo(new Vector2d(24, -58))
                .lineToSplineHeading(new Pose2d(38, -38, Math.toRadians(180)))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(38, -12))
                .lineToSplineHeading(new Pose2d(-56, -12, Math.toRadians(180)))
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(38, -12, Math.toRadians(180)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        driveTrain.followTrajectorySequence(trajSeq);

        PoseStorage.hasAutoRun = true;
        PoseStorage.currentPose = driveTrain.getPoseEstimate();
    }
}
