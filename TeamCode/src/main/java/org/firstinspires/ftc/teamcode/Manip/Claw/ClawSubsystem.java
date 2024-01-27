package org.firstinspires.ftc.teamcode.Manip.Claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Wrist.WristSubsystem;

public class ClawSubsystem extends SubsystemBase {
    private final Servo claw1, claw2;
    private int clawNum;

    public ClawSubsystem(final HardwareMap hMap, String servo1, String servo2){
        claw1 = hMap.get(Servo.class, servo1);
        claw2 = hMap.get(Servo.class, servo2);
    }

    public void moveClawOne() {
        double pos = getClawOnePos();
        if (pos == 1.0) {
            claw1.setPosition(0.0);
        } else claw1.setPosition(1.0);
    }
    public void moveClawTwo() {
        double pos = getClawTwoPos();
        if (pos == 1.0) {
            claw2.setPosition(0.0);
        } else claw2.setPosition(1.0);
    }

    public double getClawOnePos() {
        return claw1.getPosition();
    }

    public double getClawTwoPos() {
        return claw2.getPosition();
    }

    public void autoMoveClaw(WristSubsystem wrist, ShoulderSubsystem shoulder) {
        if (shoulder.getEncoderValue() <= 1550) {
            if (wrist.getPos() <= 0.1) {
                clawNum = 2;
            } else if (wrist.getPos() >= 0.9) {
                clawNum = 1;
            }
        } else {
            if (wrist.getPos() <= 0.1) {
                clawNum = 1;
            } else if (wrist.getPos() >= 0.9) {
                clawNum = 2;
            }
        }

        if (clawNum == 1) {
            moveClawOne();
        } else if (clawNum == 2) {
            moveClawTwo();
        }
    }

    public void clawOneToPos(double pos) {
        claw1.setPosition(pos);
    }
    public void clawTwoToPos(double pos) {
        claw2.setPosition(pos);
    }
}
