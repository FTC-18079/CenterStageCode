package org.firstinspires.ftc.teamcode.Arm;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftToPos;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderToPos;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowToPos;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ArmCommand extends ParallelCommandGroup {
    private final DoubleSupplier shoulderVel, liftVel;

    public ArmCommand(ShoulderSubsystem shoulder, LiftSubsystem lift, StowSubsystem stow, IntSupplier shoulderPos, IntSupplier liftPos, DoubleSupplier stowPos, Telemetry tele) {
        if (liftPos.getAsInt() == ArmConstants.LIFT_POS_REST) liftVel = () -> ArmConstants.LIFT_VEL_TO_STOW;
        else liftVel = () -> ArmConstants.LIFT_VELOCITY;

        shoulderVel = () -> ArmConstants.SHOULDER_VELOCITY;

        addCommands(new ShoulderToPos(shoulder, shoulderPos, shoulderVel, tele), new LiftToPos(lift, liftPos, liftVel, tele), new StowToPos(stow, stowPos));
        addRequirements(shoulder, lift, stow);
    }
}
