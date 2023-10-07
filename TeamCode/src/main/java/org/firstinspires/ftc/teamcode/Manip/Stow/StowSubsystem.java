package org.firstinspires.ftc.teamcode.Manip.Stow;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StowSubsystem extends SubsystemBase {
    private Servo stow;

    public StowSubsystem(HardwareMap hMap, String name) {
        stow = hMap.get(Servo.class, name);
    }

    public void moveStow() {
        double pos = stow.getPosition();
        if (pos == 1.0) {
            stow.setPosition(0.0);
        } else stow.setPosition(1.0);
    }

    public void stow() {
        stow.setPosition(0.35);
    }

    public void down() {
        stow.setPosition(1.0);
    }
}
