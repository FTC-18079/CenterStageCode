package org.firstinspires.ftc.teamcode.Manip;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.Stow;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Wrist.WristCommand;
import org.firstinspires.ftc.teamcode.Manip.Wrist.WristSubsystem;

import java.util.function.IntSupplier;

public class WristStow extends SequentialCommandGroup {
    public WristStow(WristSubsystem wrist, StowSubsystem stow, double shoulderPos) {
        if (shoulderPos > 85) {
            addCommands(
                    new WristCommand(wrist)
            );
        } else {
            addCommands(
                    new Stow(stow),
                    new WristCommand(wrist)
            );
        }

        addRequirements(wrist, stow);
    }
}
