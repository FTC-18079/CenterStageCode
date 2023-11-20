package org.firstinspires.ftc.teamcode.Manip.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

public class CloseClawOne extends CommandBase implements Runnable {
    private final ClawSubsystem clawOne;

    public CloseClawOne(ClawSubsystem clawOne){
        this.clawOne = clawOne;
        addRequirements(clawOne);
    }
    @Override
    public void initialize(){
        clawOne.clawOneToPos(0.0);
    }
    @Override
    public void run() {
        initialize();
    }
    @Override
    public boolean isFinished(){return true;}
}
