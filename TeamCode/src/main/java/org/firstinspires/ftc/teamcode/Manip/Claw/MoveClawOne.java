package org.firstinspires.ftc.teamcode.Manip.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

public class MoveClawOne extends CommandBase implements Runnable{

    private final ClawSubsystem clawOne;

    public MoveClawOne(ClawSubsystem clawOne){
        this.clawOne = clawOne;
        addRequirements(clawOne);
    }
    @Override
    public void run(){
        initialize();
    }
    @Override
    public void initialize(){
        clawOne.moveClawOne();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
