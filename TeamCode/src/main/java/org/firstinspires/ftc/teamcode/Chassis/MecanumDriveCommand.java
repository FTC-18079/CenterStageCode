package org.firstinspires.ftc.teamcode.Chassis;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {
    private final MecanumDrive drive;
    private final DoubleSupplier leftY, leftX, rightX;

    public MecanumDriveCommand(MecanumDrive drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble());
    }
}
