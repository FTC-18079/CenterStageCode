package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;

public class ShooterServoCommand extends CommandBase implements Runnable {
    private final ShooterSubsystem shooterServo;

    private final double desiredPos;

    public ShooterServoCommand(ShooterSubsystem shooterServo, double pos) {
        this.shooterServo = shooterServo;
        this.desiredPos = pos;
        addRequirements(shooterServo);
    }

    @Override
    public void initialize() {
        shooterServo.shooterToPos(desiredPos);
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
