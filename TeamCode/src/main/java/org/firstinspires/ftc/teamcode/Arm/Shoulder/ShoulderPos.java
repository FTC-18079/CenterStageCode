package org.firstinspires.ftc.teamcode.Arm.Shoulder;

import com.arcrobotics.ftclib.command.CommandBase;

public class ShoulderPos extends CommandBase {
    private final ShoulderSubsystem shoulder;

    public ShoulderPos(ShoulderSubsystem subsystem) {
        shoulder = subsystem;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        shoulder.getEncoderValue();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
