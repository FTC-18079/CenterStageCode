package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterServoSubsystem extends SubsystemBase {
    private final Servo shooterservo;

    public ShooterServoSubsystem(final HardwareMap hMap, String servo){
        shooterservo = hMap.get(Servo.class, servo);
    }

    public void moveshootservo(){
        double pos = getshooterservopos();
        if (pos == 1.0){
            shooterservo.setPosition(0.0);
        }else {
            shooterservo.setPosition(1.0);
        }
    }
    public double getshooterservopos(){
        return shooterservo.getPosition();
    }
    public void shooterToPos(double pos) {
        shooterservo.setPosition(pos);
    }
}
