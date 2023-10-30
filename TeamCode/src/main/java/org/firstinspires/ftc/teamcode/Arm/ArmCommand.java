package org.firstinspires.ftc.teamcode.Arm;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftToPos;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderToPos;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowToPos;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ArmCommand extends ParallelCommandGroup {
    private final DoubleSupplier shoulderVel, liftVel;
    private final Telemetry tele;

    public ArmCommand(ShoulderSubsystem shoulder, LiftSubsystem lift, StowSubsystem stow, IntSupplier shoulderPos, IntSupplier liftPos, DoubleSupplier stowPos, Telemetry tele) {
        shoulderVel = () -> Constants.SHOULDER_VELOCITY;
        liftVel = () -> Constants.LIFT_VELOCITY;
        this.tele = tele;

        addCommands(new ShoulderToPos(shoulder, shoulderPos, shoulderVel, tele), new LiftToPos(lift, liftPos, liftVel, tele), new StowToPos(stow, stowPos));
        addRequirements(shoulder, lift, stow);
    }
}
