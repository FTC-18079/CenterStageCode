package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.VisionSubsystem;

@Autonomous(name = "Test Auto", group = "Tests")
public class AutoTest extends CommandOpMode {

    VisionSubsystem visionSubsystem;

    @Override
    public void initialize() {
        visionSubsystem.enableTfod();

        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap, telemetry);
        chassis.setPoseEstimate(new Pose2d(12, -63.339, Math.toRadians(90)));
        chassis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}