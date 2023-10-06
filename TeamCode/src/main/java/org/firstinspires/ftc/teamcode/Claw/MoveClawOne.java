package org.firstinspires.ftc.teamcode.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

public class MoveClawOne extends CommandBase {
    private final ClawSubsystem claw;

    public MoveClawOne(ClawSubsystem subsystem){
        claw = subsystem;
        addRequirements(claw);
    }

    @Override
    public void initialize(){
        claw.moveClawOne();
    }
}
