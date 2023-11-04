package org.firstinspires.ftc.teamcode.Telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class TelemetrySS extends SubsystemBase {
    private final Telemetry m_telemetry;

    public TelemetrySS(Telemetry tele) {
        m_telemetry = tele;
    }

    public void run(double[] values) {
        for (int i = 0; i < values.length; i++) {
            m_telemetry.addData(Constants.telemetryData[i], values[i]);
        }
        m_telemetry.addData("Test", values);
        m_telemetry.update();
    }
}
