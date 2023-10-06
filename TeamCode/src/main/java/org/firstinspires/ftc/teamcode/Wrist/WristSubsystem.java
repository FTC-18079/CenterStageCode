package org.firstinspires.ftc.teamcode.Wrist;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase {

    private final Servo wrist;

    public WristSubsystem(HardwareMap hMap, String S1){
        wrist = hMap.get(Servo.class, S1);
    }

    public void moveWrist(){
        double pose = wrist.getPosition();
        switch((int) pose){
            case 1:
                wrist.setPosition(0);
                break;
            case 0:
                wrist.setPosition(1);
                break;
            default:
                wrist.setPosition(0);

        }
    }

}
