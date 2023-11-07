package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;

public class ShooterServoCommand extends CommandBase implements Runnable {
    private final ShooterServoSubsystem shooterServo;

    public ShooterServoCommand(ShooterServoSubsystem shooterServo) {
        this.shooterServo = shooterServo;
        addRequirements(shooterServo);
    }

    @Override
    public void initialize() {
        shooterServo.shooterToPos(1);
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
