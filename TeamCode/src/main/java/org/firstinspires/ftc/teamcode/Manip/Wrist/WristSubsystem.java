package org.firstinspires.ftc.teamcode.Manip.Wrist;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase {
    private final Servo wrist;
    private final DcMotorEx encoder;

    public WristSubsystem(HardwareMap hMap, String name, String encoder) {
        wrist = hMap.get(Servo.class, name);
        this.encoder = hMap.get(DcMotorEx.class, encoder);
        this.encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveWrist() {
        double wristPos = wrist.getPosition();
        if (wristPos == 1.0) {
            wrist.setPosition(0.0);
        } else wrist.setPosition(1.0);
    }

    public double getPos() {
        return wrist.getPosition();
    }

    public double getEncoderPos() {
        return encoder.getCurrentPosition();
    }
}
