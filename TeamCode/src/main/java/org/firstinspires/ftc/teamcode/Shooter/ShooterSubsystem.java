package org.firstinspires.ftc.teamcode.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx shooter;
    private final Servo shooterServo;
    public static double RPS = 1.0;
    public static double kP = 4.0;
    public static double kI = 0.0;
    public static double kD = 0.5;
    private final PIDController pidController = new PIDController(kP, kI, kD);
    private double output;
    public ShooterSubsystem(final HardwareMap hMap, String motor, String servo) {
        shooter = hMap.get(DcMotorEx.class, motor);
        shooterServo = hMap.get(Servo.class, servo);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double power) {
        shooter.setPower(power);
    }//0.74

    public void drive() {
        pidController.setSetPoint(RPS * 360.0);
        output = pidController.calculate(getVelocity());
        shooter.setVelocity(output, AngleUnit.DEGREES);
    }

    public boolean reachedTargetVel() {
        return Math.abs((RPS * 360.0) - getVelocity()) < 5;
    }

    public double getOutput() {
        return output;
    }

    public double getVelocity() {
        return shooter.getVelocity(AngleUnit.DEGREES);
    }

    public void stop() {
        shooter.setPower(0);
    }

    public void shooterToPos(double pos) {
        shooterServo.setPosition(pos);
    }
}