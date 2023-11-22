package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterServoCommand extends CommandBase implements Runnable {
    private final ShooterSubsystem shooterServo;

    public ShooterServoCommand(ShooterSubsystem shooterServo) {
        this.shooterServo = shooterServo;
        addRequirements(shooterServo);
    }

    @Override
    public void initialize() {
        shooterServo.shooterToPos(0);
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
