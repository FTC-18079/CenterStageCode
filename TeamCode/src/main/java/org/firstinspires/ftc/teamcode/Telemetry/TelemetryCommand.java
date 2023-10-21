package org.firstinspires.ftc.teamcode.Telemetry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class TelemetryCommand extends CommandBase {
    public TelemetrySS m_telemetry;
//    private final ArrayList<DoubleSupplier> values = new ArrayList<>();
    private final double[] values = new double[Constants.telemetryData.length];

    public TelemetryCommand(TelemetrySS subsystem, DoubleSupplier... values) {
        m_telemetry = subsystem;
        for (int i=0; i<values.length; i++) {
            this.values[i] = values[i].getAsDouble();
        }
        addRequirements(m_telemetry);
    }

    @Override
    public void execute() {
        m_telemetry.run(values);
    }
}
