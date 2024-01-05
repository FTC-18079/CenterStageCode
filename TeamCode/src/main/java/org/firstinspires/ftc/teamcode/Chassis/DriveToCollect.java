package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveToCollect extends CommandBase {
    private final MecanumDrive drive;
    private final DoubleSupplier targetX, targetY, targetAngle;

    private PIDFController pidController;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(1.5, 0.1, 0.5);

    public DriveToCollect(MecanumDrive drive, DoubleSupplier targetX, DoubleSupplier targetY, DoubleSupplier targetAngle) {
        this.drive = drive;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;

        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        pidController = new PIDFController(pidCoefficients);
        pidController.setInputBounds(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPoseEstimate();

        pidController.setTargetPosition(targetAngle.getAsDouble());
        double headingControl = pidController.update(currentPose.getHeading());

        drive.drive(targetY.getAsDouble(), targetX.getAsDouble(), headingControl, 0);
        drive.update();
    }
}
