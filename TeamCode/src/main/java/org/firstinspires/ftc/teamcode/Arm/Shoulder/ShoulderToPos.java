package org.firstinspires.ftc.teamcode.Arm.Shoulder;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ShoulderToPos extends CommandBase {
    private final ShoulderSubsystem shoulder;
    private final IntSupplier position;
    private final DoubleSupplier velocity;

    public ShoulderToPos(ShoulderSubsystem subsystem, IntSupplier pos, DoubleSupplier vel) {
        shoulder = subsystem;
        position = pos;
        velocity = vel;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        shoulder.moveToPos(
                position.getAsInt(),
                velocity.getAsDouble()
        );
    }

    @Override
    public boolean isFinished() {
        return !shoulder.isRunningEnc();
    }
}
