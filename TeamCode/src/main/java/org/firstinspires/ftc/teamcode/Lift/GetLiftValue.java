package org.firstinspires.ftc.teamcode.Lift;

import com.arcrobotics.ftclib.command.CommandBase;

public class GetLiftValue extends CommandBase {
    private final LiftSubsystem lift;

    public GetLiftValue (LiftSubsystem subsystem) {
        lift = subsystem;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.getEncoderValue();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
