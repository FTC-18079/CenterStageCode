package org.firstinspires.ftc.teamcode.legacyclaw;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LegacyClaw extends SubsystemBase {
    boolean clawOneActive;
    Servo stow;
    Servo wrist;
    Servo claw1;
    Servo claw2;

    public static double stowPos = 0.45;
    public static double downPos = 0.87;
    public static double openPos = 0.55;

    public LegacyClaw(HardwareMap hMap) {
        stow = hMap.get(Servo.class, "stow");
        wrist = hMap.get(Servo.class, "wrist");
        claw1 = hMap.get(Servo.class, "clawOne");
        claw2 = hMap.get(Servo.class, "clawTwo");

        clawOneActive = true;
    }

    public void grab() {
        if (clawOneActive) claw1.setPosition(0);
        else claw2.setPosition(0);
    }

    public void release() {
        if (clawOneActive) claw1.setPosition(openPos);
        else claw2.setPosition(openPos);
    }

    public void toggleStow() {
        if (stow.getPosition() == downPos) stowUp();
        else stowDown();
    }

    public void stowDown() {
        stow.setPosition(downPos);
    }

    public void stowUp() {
        stow.setPosition(stowPos);
    }

    public void setStow(double pos) {
        stow.setPosition(pos);
    }

    public void rotate() {
        double wristPos = wrist.getPosition();
        if (wristPos == 1.0) {
            wrist.setPosition(0.0);
        } else wrist.setPosition(1.0);
        clawOneActive = ! clawOneActive;
    }
}
