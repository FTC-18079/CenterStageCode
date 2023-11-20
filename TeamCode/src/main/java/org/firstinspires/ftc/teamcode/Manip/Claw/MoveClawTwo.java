package org.firstinspires.ftc.teamcode.Manip.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

public class MoveClawTwo extends CommandBase implements Runnable {

    private final ClawSubsystem clawTwo;

    public MoveClawTwo(ClawSubsystem clawTwo){
        this.clawTwo = clawTwo;
        addRequirements(clawTwo);
    }

    @Override
    public void initialize(){
        clawTwo.moveClawTwo();
    }
    @Override
    public void run(){
        initialize();
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
