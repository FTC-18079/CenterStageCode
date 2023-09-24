package org.firstinspires.ftc.teamcode.Arm.Shoulder;

import com.arcrobotics.ftclib.command.CommandBase;

public class ResetEncoder extends CommandBase {
    private final ShoulderSubsystem shoulder;

    public ResetEncoder (ShoulderSubsystem subsystem) {
        shoulder = subsystem;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        shoulder.resetLimit();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
