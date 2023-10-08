package org.firstinspires.ftc.teamcode.Arm;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftToPos;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderToPos;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ArmCommand extends ParallelCommandGroup {
    private final DoubleSupplier shoulderVel, liftVel;
    public ArmCommand(ShoulderSubsystem shoulder, LiftSubsystem lift, IntSupplier shoulderPos, IntSupplier liftPos) {
        shoulderVel = () -> Constants.SHOULDER_VELOCITY;
        liftVel = () -> Constants.LIFT_VELOCITY;

        addCommands(
                new ShoulderToPos(shoulder, shoulderPos, shoulderVel),
                new LiftToPos(lift, liftPos, liftVel)
        );
        addRequirements(shoulder, lift);
    }
}
