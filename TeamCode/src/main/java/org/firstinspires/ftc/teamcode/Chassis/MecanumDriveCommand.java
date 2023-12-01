package org.firstinspires.ftc.teamcode.Chassis;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {
    private final MecanumDrive drive;
    private final DoubleSupplier leftY, leftX, rightX;
    private final DoubleSupplier brakePower;

    public MecanumDriveCommand(MecanumDrive drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, DoubleSupplier brakePower) {
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        this.brakePower = brakePower;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble(), brakePower.getAsDouble());
        drive.update();
    }
}
