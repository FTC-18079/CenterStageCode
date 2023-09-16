package org.firstinspires.ftc.teamcode.Telemetry;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class TelemetryCommand extends CommandBase {
    public TelemetrySS m_telemetry;
    private final DoubleSupplier lift;

    public TelemetryCommand(TelemetrySS subsystem, DoubleSupplier value1) {
        m_telemetry = subsystem;
        lift = value1;
        addRequirements(m_telemetry);
    }

    @Override
    public void execute() {
        m_telemetry.run(
                lift.getAsDouble()
        );
    }
}
