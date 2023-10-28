package org.firstinspires.ftc.teamcode.Arm.Lift;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class LiftToPos extends CommandBase implements Runnable {
    private final LiftSubsystem lift;
    private final IntSupplier position;
    private final DoubleSupplier velocity;

    public LiftToPos(LiftSubsystem subsystem, IntSupplier pos, DoubleSupplier vel) {
        lift = subsystem;
        position = pos;
        velocity = vel;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.moveToPos(
                position.getAsInt(),
                velocity.getAsDouble()
        );
    }

    @Override
    public boolean isFinished() {
        return !lift.isRunningEnc();
    }

    @Override
    public void run() {
        initialize();
    }
}
