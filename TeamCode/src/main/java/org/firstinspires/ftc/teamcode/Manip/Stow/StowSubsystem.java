package org.firstinspires.ftc.teamcode.Manip.Stow;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@Config
public class StowSubsystem extends SubsystemBase {
    private Servo stow;
    private static double pos = 0.0;

    public StowSubsystem(HardwareMap hMap, String name) {
        stow = hMap.get(Servo.class, name);
    }

    public void moveStow() {
        pos = getPos();
        if (pos == 1.0) {
            pos = 0.0;
//            stow.setPosition(0.0);
        } else pos = 1.0; /*stow.setPosition(1.0);*/
    }

    public void toPos(double pos) {
        stow.setPosition(pos);
    }

    public void stow() {
        pos = 0.15;
//        stow.setPosition(0.15);
    }

    public void down() {
        pos = Constants.STOW_POS_REST;
//        stow.setPosition(Constants.STOW_POS_REST);
    }

    public double getPos() {
        return stow.getPosition();
    }

    @Override
    public void periodic() {
        stow.setPosition(pos);
    }
}
