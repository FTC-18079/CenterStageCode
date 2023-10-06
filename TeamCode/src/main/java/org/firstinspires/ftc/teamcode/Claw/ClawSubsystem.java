package org.firstinspires.ftc.teamcode.Claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    private final Servo claw1, claw2;

    public ClawSubsystem(final HardwareMap hMap, String S1, String S2){
        claw1 = hMap.get(Servo.class, S1);
        claw2 = hMap.get(Servo.class, S2);

    }

    public void moveClawOne(){
        double pose = claw1.getPosition();
        switch ((int) pose){
            case 1:
                claw1.setPosition(0);
                break;
            case 0:
                claw1.setPosition(1);
                break;
            default:
                claw1.setPosition(0);
                break;
        }


    }
    public void moveClawTwo(){
        double pose = claw2.getPosition();
        switch ((int) pose){
            case 1:
                claw2.setPosition(0);
            case 0:
                claw2.setPosition(1);
            default:
                claw2.setPosition(0);
        }
    }

}
