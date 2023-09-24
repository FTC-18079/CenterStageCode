package org.firstinspires.ftc.teamcode.Arm.Shoulder;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ShoulderCommand extends CommandBase {
    private final ShoulderSubsystem shoulder;
    private final DoubleSupplier velocity;

    public ShoulderCommand (ShoulderSubsystem subsystem, DoubleSupplier vel) {
        shoulder = subsystem;
        velocity = vel;
        addRequirements(shoulder);
    }

    @Override
    public void execute() {
        shoulder.drive(velocity.getAsDouble());
    }
}
