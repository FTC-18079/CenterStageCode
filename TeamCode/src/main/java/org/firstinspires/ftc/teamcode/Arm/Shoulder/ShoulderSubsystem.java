package org.firstinspires.ftc.teamcode.Arm.Shoulder;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShoulderSubsystem extends SubsystemBase {
    private final DcMotorEx shoulder1, shoulder2;

    public ShoulderSubsystem(final HardwareMap hMap, String motor1, String motor2) {
        shoulder1 = hMap.get(DcMotorEx.class, motor1);
        shoulder2 = hMap.get(DcMotorEx.class, motor2);

        shoulder1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shoulder2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shoulder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulder1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoulder2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shoulder2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double sup) {
        shoulder1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder1.setPower(sup);
        shoulder2.setPower(sup);
    }

    public void moveToPos(int target, double vel) {
        shoulder1.setTargetPosition(target);
        shoulder2.setTargetPosition(target);
        shoulder1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder1.setVelocity(vel);
        shoulder2.setVelocity(vel);
    }

    public boolean isRunningEnc() {
        return shoulder1.isBusy();
    }

    public int getEncoderValue() {
        return shoulder1.getCurrentPosition();
    }

    public void resetLimit() {
        shoulder1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoulder2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
