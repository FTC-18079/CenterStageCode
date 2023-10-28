package org.firstinspires.ftc.teamcode.Manip.Stow;

import com.arcrobotics.ftclib.command.CommandBase;

public class Down extends CommandBase implements Runnable{
    private final StowSubsystem stow;

    public Down(StowSubsystem subsystem) {
        stow = subsystem;
        addRequirements(stow);
    }

    @Override
    public void initialize() {
        stow.down();
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
