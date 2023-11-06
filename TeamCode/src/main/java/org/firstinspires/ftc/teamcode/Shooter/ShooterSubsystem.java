package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx shooter;

    public ShooterSubsystem(final HardwareMap hMap, String motor) {
        shooter = hMap.get(DcMotorEx.class, motor);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive() {
        shooter.setPower(0.635);
    }

    public void stop() {
        shooter.setPower(0);
    }
}