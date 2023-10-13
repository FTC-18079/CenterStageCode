package org.firstinspires.ftc.teamcode.Manip.Stow;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class StowSubsystem extends SubsystemBase {
    private Servo stow;

    public StowSubsystem(HardwareMap hMap, String name) {
        stow = hMap.get(Servo.class, name);
    }

    public void moveStow() {
        double pos = getPos();
        if (pos == 1.0) {
            stow.setPosition(0.0);
        } else stow.setPosition(1.0);
    }

    public void toPos(double pos) {
        stow.setPosition(pos);
    }

    public void stow() {
        stow.setPosition(0.35);
    }

    public void down() {
        stow.setPosition(Constants.STOW_POS_REST);
    }

    public double getPos() {
        return stow.getPosition();
    }
}
