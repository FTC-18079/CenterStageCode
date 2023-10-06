package org.firstinspires.ftc.teamcode.Wrist;

import com.arcrobotics.ftclib.command.CommandBase;

public class WristCommand extends CommandBase {
     private final WristSubsystem wrist;

     public WristCommand(WristSubsystem subsystem){
         wrist = subsystem;
         addRequirements(wrist);
     }

     @Override
     public void initialize(){
         wrist.moveWrist();
     }

     @Override
    public boolean isFinished() {
         return true;
     }
}
