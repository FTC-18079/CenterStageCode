package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class NewDriveCommand extends CommandBase {
    private final MecanumDrive drive;

    private final DoubleSupplier leftY, leftX, rightX;
    public NewDriveCommand(MecanumDrive chassis, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drive = chassis;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
    }

    @Override
    public void execute() {
        drive.drive(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble(), 0);
    }
}
