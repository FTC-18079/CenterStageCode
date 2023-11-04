package org.firstinspires.ftc.teamcode.Manip.Stow;

import com.arcrobotics.ftclib.command.CommandBase;

public class Stow extends CommandBase implements Runnable {
    private final StowSubsystem stow;

    public Stow(StowSubsystem subsystem) {
        stow = subsystem;
        addRequirements(stow);
    }

    @Override
    public void initialize() {
        stow.stow();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void run() {
        initialize();
    }
}
