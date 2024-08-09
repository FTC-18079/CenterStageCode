package org.firstinspires.ftc.teamcode.legacyclaw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LegacyClaw extends SubsystemBase {
    boolean clawOneActive;
    Servo stow;
    Servo wrist;
    Servo claw1;
    Servo claw2;

    public LegacyClaw(HardwareMap hMap) {
        stow = hMap.get(Servo.class, "stow");
        wrist = hMap.get(Servo.class, "wrist");
        claw1 = hMap.get(Servo.class, "clawOne");
        claw2 = hMap.get(Servo.class, "clawTwo");

        clawOneActive = true;
    }

    public void grab() {
        if (clawOneActive) claw1.setPosition(0);
        else claw2.setPosition(1);
    }

    public void release() {
        if (clawOneActive) claw1.setPosition(1);
        else claw2.setPosition(0);

    }

    public void stowDown() {
        stow.setPosition(1);
    }

    public void stowUp() {
        stow.setPosition(0);
    }

    public void rotate() {
        double wristPos = wrist.getPosition();
        if (wristPos == 1.0) {
            wrist.setPosition(0.0);
        } else wrist.setPosition(1.0);
        clawOneActive = ! clawOneActive;
    }
}
