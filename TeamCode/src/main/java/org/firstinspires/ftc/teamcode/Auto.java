package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.RRCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Auto", group = "Tests")
public class Auto extends LinearOpMode {
    MecanumDrive drive;
    LiftSubsystem lift;
    RunCommand runCommand;

    @Override
    public void runOpMode() {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap, telemetry);

        Pose2d startPose = new Pose2d(36.0,-62.50, Math.toRadians(90));

        driveTrain.setPoseEstimate(startPose);

        Trajectory traj1 = driveTrain.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, -26.5), Math.toRadians(90))
                .build();

        Trajectory traj2 = driveTrain.trajectoryBuilder(traj1.end())
                .strafeRight(12)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        driveTrain.followTrajectory(traj1);
        driveTrain.followTrajectory(traj2);
    }
}
