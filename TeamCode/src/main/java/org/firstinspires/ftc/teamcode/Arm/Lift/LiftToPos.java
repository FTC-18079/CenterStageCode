package org.firstinspires.ftc.teamcode.Arm.Lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class LiftToPos extends CommandBase implements Runnable {
    private final LiftSubsystem lift;
    private final IntSupplier position;
    private final DoubleSupplier velocity;
    private final Telemetry tele;

    private final Timing.Timer timer = new Timing.Timer(3);

    public LiftToPos(LiftSubsystem subsystem, IntSupplier pos, DoubleSupplier vel, Telemetry tele) {
        lift = subsystem;
        position = pos;
        velocity = vel;
        this.tele = tele;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        timer.start();
        lift.moveToPos(
                position.getAsInt(),
                velocity.getAsDouble()
        );
    }

    @Override
    public boolean isFinished() {
        return !lift.isRunning() || timer.done();
    }

    @Override
    public void execute() {
        if (lift.getTouch() && lift.getEncoderValue() < lift.getTargetPos()) lift.resetLimit();
        tele.addData("Lift Encoder:", lift.getEncoderValue());
    }

    @Override
    public void run() {
        initialize();
    }
}
