package org.firstinspires.ftc.teamcode.Manip.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Wrist.WristSubsystem;

public class AutoMoveClaw extends CommandBase {
    private ClawSubsystem claw;
    private WristSubsystem wrist;
    private ShoulderSubsystem shoulder;

    public AutoMoveClaw(ClawSubsystem subsystem, WristSubsystem wrist, ShoulderSubsystem shoulder) {
        claw = subsystem;
        this.wrist = wrist;
        this.shoulder = shoulder;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.autoMoveClaw(wrist, shoulder);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
