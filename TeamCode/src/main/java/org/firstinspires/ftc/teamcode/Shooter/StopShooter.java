package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;

public class StopShooter extends CommandBase implements Runnable {
    private final ShooterSubsystem shooter;

    public StopShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void run() {
        initialize();
    }
}
