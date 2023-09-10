package org.firstinspires.ftc.teamcode.Lift;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LiftCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final DoubleSupplier velocity;
    private final BooleanSupplier limitEnabled;

    public LiftCommand (LiftSubsystem subsystem, DoubleSupplier vel, BooleanSupplier limE) {
        lift = subsystem;
        velocity = vel;
        limitEnabled = limE;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.drive(
                velocity.getAsDouble(),
                limitEnabled.getAsBoolean()
        );
    }
}
