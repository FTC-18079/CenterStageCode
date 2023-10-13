package org.firstinspires.ftc.teamcode.Manip.Stow;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class StowToPos extends CommandBase {
    private StowSubsystem stow;
    DoubleSupplier pos;

    public StowToPos(StowSubsystem subsystem, DoubleSupplier targetPos) {
        stow = subsystem;
        pos = targetPos;
        addRequirements(stow);
    }

    @Override
    public void initialize() {
        stow.toPos(pos.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
