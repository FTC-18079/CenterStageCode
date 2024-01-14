package org.firstinspires.ftc.teamcode.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx shooter;
    private final Servo shooterServo;
    public ShooterSubsystem(final HardwareMap hMap, String motor, String servo) {
        shooter = hMap.get(DcMotorEx.class, motor);
        shooterServo = hMap.get(Servo.class, servo);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double power) {
        shooter.setPower(power);
    }//0.74

    public void stop() {
        shooter.setPower(0);
    }

    public void shooterToPos(double pos) {
        shooterServo.setPosition(pos);
    }
}