package org.firstinspires.ftc.teamcode.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

public class MoveClawTwo extends CommandBase {
    private final ClawSubsystem claw;

    public MoveClawTwo(ClawSubsystem subsystem){
        claw = subsystem;
        addRequirements(claw);
    }

    @Override
    public void initialize(){
        claw.moveClawTwo();
    }

    public boolean isFinished() {
        return true;
    }
}
