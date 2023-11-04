package org.firstinspires.ftc.teamcode.Manip.Stow;

import com.arcrobotics.ftclib.command.CommandBase;

public class StowCommand extends CommandBase {
    private final StowSubsystem stow;

    public StowCommand(StowSubsystem subsystem) {
        stow = subsystem;
        addRequirements(stow);
    }

    @Override
    public void execute() {
        stow.toPos(stow.getPos());
    }
}
