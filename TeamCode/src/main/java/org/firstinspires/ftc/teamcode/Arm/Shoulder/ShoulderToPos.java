package org.firstinspires.ftc.teamcode.Arm.Shoulder;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ShoulderToPos extends CommandBase implements Runnable {
    private final ShoulderSubsystem shoulder;
    private final IntSupplier position;
    private final DoubleSupplier velocity;
    private final Telemetry tele;

    public ShoulderToPos(ShoulderSubsystem subsystem, IntSupplier pos, DoubleSupplier vel, Telemetry tele) {
        shoulder = subsystem;
        position = pos;
        velocity = vel;
        this.tele = tele;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        shoulder.moveToPos(
                position.getAsInt(),
                velocity.getAsDouble()
        );
    }

    @Override
    public boolean isFinished() {
        return !shoulder.isRunning();
    }

    @Override
    public void execute() {
//        if (shoulder.getTouch()) shoulder.resetLimit();
        tele.addData("Shoulder Encoder:", shoulder.getEncoderValue());
    }

    @Override
    public void run() {
        initialize();
    }
}
