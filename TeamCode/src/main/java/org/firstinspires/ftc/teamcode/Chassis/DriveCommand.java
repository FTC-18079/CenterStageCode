package org.firstinspires.ftc.teamcode.Chassis;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private Chassis chassis;

    private final DoubleSupplier strafe, forward, rotation, gyro;
    private final BooleanSupplier isRobotOriented;

    public DriveCommand(Chassis subsystem, DoubleSupplier str, DoubleSupplier fwd, DoubleSupplier rot, DoubleSupplier gyr, BooleanSupplier isro){
        chassis = subsystem;
        strafe = str;
        forward = fwd;
        rotation = rot;
        gyro = gyr;
        isRobotOriented = isro;

        addRequirements(chassis);
    }
    @Override
    public void execute(){
        chassis.drive(strafe.getAsDouble(), forward.getAsDouble(), rotation.getAsDouble(), gyro.getAsDouble(), isRobotOriented.getAsBoolean());
    }


}
