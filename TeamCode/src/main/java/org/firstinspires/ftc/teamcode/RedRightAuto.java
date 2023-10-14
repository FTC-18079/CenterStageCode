package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.RRCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedRightAuto", group = "Autos")
public class RedRightAuto extends LinearOpMode {
    LiftSubsystem lift;
    RunCommand runCommand;

    @Override
    public void runOpMode() {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap, telemetry);

        Pose2d startPose = new Pose2d(12, -61.5, Math.toRadians(90));

        driveTrain.setPoseEstimate(startPose);
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PoseStorage.currentPose = driveTrain.getPoseEstimate();

        TrajectorySequence trajSeq = driveTrain.trajectorySequenceBuilder(startPose)
                .forward(26)
                .waitSeconds(0.25)
                .back(16)
                .splineToSplineHeading(new Pose2d(44, -35, Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(14, -60), Math.toRadians(180))
                .forward(30)
                .splineToConstantHeading(new Vector2d(-55.5, -35), Math.toRadians(90))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        driveTrain.followTrajectorySequence(trajSeq);

        PoseStorage.currentPose = driveTrain.getPoseEstimate();
    }
}
