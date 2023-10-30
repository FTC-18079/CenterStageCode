package org.firstinspires.ftc.teamcode.Manip.Wrist;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase {
    private final Servo wrist;

    public WristSubsystem(HardwareMap hMap, String name) {
        wrist = hMap.get(Servo.class, name);
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

    public void toPos(double pos) {
        wrist.setPosition(pos);
    }
}
