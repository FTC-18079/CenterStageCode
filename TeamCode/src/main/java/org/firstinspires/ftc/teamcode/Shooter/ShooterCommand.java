package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class ShooterCommand extends SequentialCommandGroup {
    private final Telemetry tele;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier power, Telemetry tele){
        this.tele = tele;
        addCommands(
                new FireShooter(shooterSubsystem, power),
                new WaitCommand(1200),
                new ShooterServoCommand(shooterSubsystem,0),
                new WaitCommand(1700),
                new StopShooter(shooterSubsystem),
                new ShooterServoCommand(shooterSubsystem, 1)
        );
        addRequirements(shooterSubsystem);
    }

}
