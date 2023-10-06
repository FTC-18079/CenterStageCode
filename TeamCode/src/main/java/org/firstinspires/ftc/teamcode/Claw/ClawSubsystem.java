package org.firstinspires.ftc.teamcode.Claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    private final Servo claw1, claw2;

    public ClawSubsystem(final HardwareMap hMap, String servo1, String servo2){
        claw1 = hMap.get(Servo.class, servo1);
        claw2 = hMap.get(Servo.class, servo2);
    }

    public void moveClawOne(){
        double pos1 = claw1.getPosition();
        if (pos1 == 1.0) {
            claw1.setPosition(0.0);
        } else claw1.setPosition(1.0);
    }
    public void moveClawTwo(){
        double pos2 = claw2.getPosition();
        if (pos2 == 1.0) {
            claw2.setPosition(0.0);
        } else claw2.setPosition(1.0);
    }

}
