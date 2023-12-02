package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class FireShooter extends CommandBase implements Runnable {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier power;

    public FireShooter (ShooterSubsystem shooter, DoubleSupplier power) {
        this.shooter = shooter;
        this.power = power;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.drive(power.getAsDouble());
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
