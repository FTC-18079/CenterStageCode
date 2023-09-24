package org.firstinspires.ftc.teamcode.Telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySS extends SubsystemBase {
    private final Telemetry m_telemetry;

    public TelemetrySS(Telemetry tele) {
        m_telemetry = tele;
    }

    public void run(double liftValue, double shoulderValue) {
        m_telemetry.addData("Lift", liftValue);
        m_telemetry.addData("Shoulder", shoulderValue);
        m_telemetry.update();
    }
}
