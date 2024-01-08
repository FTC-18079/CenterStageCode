package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveToCollect extends CommandBase {
    private final MecanumDrive drive;
    private final DoubleSupplier leftY, leftX;
    private final Vector2d target;

    public DriveToCollect(MecanumDrive drive, DoubleSupplier leftY, DoubleSupplier leftX, Vector2d target) {
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.target = target;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        drive.headingController.setInputBounds(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        drive.collect(leftY.getAsDouble(), leftX.getAsDouble(), target);
        drive.getLocalizer().update();
    }
}
