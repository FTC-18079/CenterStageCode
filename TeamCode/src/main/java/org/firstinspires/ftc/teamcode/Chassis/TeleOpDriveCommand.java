package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrive drive;

    private final DoubleSupplier leftY, leftX, rightX, brakePower;
    private final BooleanSupplier collecting;
    private final DoubleSupplier rightTrigger;
    private final Vector2d targetPos;

    public TeleOpDriveCommand(MecanumDrive drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, DoubleSupplier brakePower, Vector2d targetPos, BooleanSupplier collecting, DoubleSupplier rightTrigger) {
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        this.brakePower = brakePower;
        this.targetPos = targetPos;
        this.collecting = collecting;
        this.rightTrigger = rightTrigger;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.driveCollect(
                leftY.getAsDouble(),
                leftX.getAsDouble(),
                rightX.getAsDouble(),
                brakePower.getAsDouble(),
                targetPos,
                collecting.getAsBoolean(),
                rightTrigger.getAsDouble()
        );
        drive.update();
    }
}
