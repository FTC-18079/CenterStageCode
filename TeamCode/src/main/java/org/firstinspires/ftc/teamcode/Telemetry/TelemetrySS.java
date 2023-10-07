package org.firstinspires.ftc.teamcode.Telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySS extends SubsystemBase {
    private final Telemetry m_telemetry;

    public TelemetrySS(Telemetry tele) {
        m_telemetry = tele;
    }

    public void run(double liftValue, double shoulderValue, double wristValue, double wristEValue, double claw1, double claw2) {
        m_telemetry.addData("Lift", liftValue);
        m_telemetry.addData("Shoulder", shoulderValue);
        m_telemetry.addData("Wrist", wristValue);
        m_telemetry.addData("Wrist Encoder", wristEValue);
        m_telemetry.addData("Claw 1 Pos", claw1);
        m_telemetry.addData("Claw 2 Pos", claw2);
        m_telemetry.update();
    }
}
