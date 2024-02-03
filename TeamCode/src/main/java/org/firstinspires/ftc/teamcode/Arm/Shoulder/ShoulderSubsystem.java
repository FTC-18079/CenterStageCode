package org.firstinspires.ftc.teamcode.Arm.Shoulder;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShoulderSubsystem extends SubsystemBase {
    private final DcMotorEx shoulder1, shoulder2;
    private final TouchSensor sensor;
    private final Telemetry tele;
    private int targetPos;
    public static double kP = 4.0;
    public static double kI = 0.0;
    public static double kD = 0.5;
    private final PIDController pidController = new PIDController(kP, kI, kD);

    public ShoulderSubsystem(final HardwareMap hMap, String motor1, String motor2, String sensor, Telemetry tele) {
        shoulder1 = hMap.get(DcMotorEx.class, motor1);
        shoulder2 = hMap.get(DcMotorEx.class, motor2);
        this.sensor = hMap.get(TouchSensor.class, sensor);
        this.tele = tele;

        shoulder1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shoulder2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shoulder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulder1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoulder2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shoulder2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double sup) {
        if(getTouch()) resetLimit();

        shoulder1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder1.setPower(sup);
        shoulder2.setPower(sup);

        tele.addData("Shoulder Encoder:", getEncoderValue());
    }

    public void moveToPos(int target) {
        targetPos = target;
        shoulder1.setTargetPosition(targetPos);
        shoulder2.setTargetPosition(targetPos);
        shoulder1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pidController.setSetPoint(target);
        double output = pidController.calculate(getEncoderValue());
        shoulder1.setVelocity(output);
        shoulder2.setVelocity(output);
    }

    public void moveToPos(int target, double vel) {
        targetPos = target;
        double damp;
        shoulder1.setTargetPosition(targetPos);
        shoulder2.setTargetPosition(targetPos);
        if (targetPos < getEncoderValue()) damp = 0.75;
        else damp = 1.0;
        shoulder1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder1.setVelocity(vel * damp);
        shoulder2.setVelocity(vel * damp);
    }

    public boolean isRunning() {
        return (Math.abs(targetPos - getEncoderValue()) > 20);
    }

    public int getEncoderValue() {
        return shoulder1.getCurrentPosition();
    }

    public boolean getTouch() {
        return sensor.isPressed();
    }

    public void resetLimit() {
        shoulder1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        if (!isRunning()) {
            shoulder1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shoulder2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void stopVelocity() {
        shoulder1.setVelocity(0);
        shoulder2.setVelocity(0);
    }
}
