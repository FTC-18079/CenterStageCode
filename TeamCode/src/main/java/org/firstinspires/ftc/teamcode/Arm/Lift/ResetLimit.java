package org.firstinspires.ftc.teamcode.Arm.Lift;

import com.arcrobotics.ftclib.command.CommandBase;

public class ResetLimit extends CommandBase {
    private final LiftSubsystem lift;

    public ResetLimit (LiftSubsystem subsystem) {
        lift = subsystem;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.resetLimit();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
