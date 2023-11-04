package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;

public class FireShooter extends CommandBase implements Runnable {
    private final ShooterSubsystem shooter;

    public FireShooter (ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.drive();
    }

    @Override
    public void run() {
        initialize();
    }
}
