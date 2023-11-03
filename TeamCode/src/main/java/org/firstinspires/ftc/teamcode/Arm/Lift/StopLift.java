package org.firstinspires.ftc.teamcode.Arm.Lift;

import com.arcrobotics.ftclib.command.CommandBase;

public class StopLift extends CommandBase implements Runnable {
    private final LiftSubsystem lift;

    public StopLift(LiftSubsystem lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.stop();
    }

    @Override
    public void run() {
        initialize();
    }
}
