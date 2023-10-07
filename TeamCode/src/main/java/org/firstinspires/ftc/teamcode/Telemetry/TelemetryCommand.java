package org.firstinspires.ftc.teamcode.Telemetry;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class TelemetryCommand extends CommandBase {
    public TelemetrySS m_telemetry;
    private final DoubleSupplier lift, shoulder, wrist, wristEnc, claw1, claw2;

    public TelemetryCommand(TelemetrySS subsystem, DoubleSupplier value1, DoubleSupplier value2, DoubleSupplier value3,
                            DoubleSupplier value4, DoubleSupplier value5, DoubleSupplier value6) {
        m_telemetry = subsystem;
        lift = value1;
        shoulder = value2;
        wrist = value3;
        wristEnc = value4;
        claw1 = value5;
        claw2 = value6;
        addRequirements(m_telemetry);
    }

    @Override
    public void execute() {
        m_telemetry.run(
                lift.getAsDouble(),
                shoulder.getAsDouble(),
                wrist.getAsDouble(),
                wristEnc.getAsDouble(),
                claw1.getAsDouble(),
                claw2.getAsDouble()
        );
    }
}
