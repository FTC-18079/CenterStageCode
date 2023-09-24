package org.firstinspires.ftc.teamcode.Arm.Shoulder;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    private final DcMotorEx shoulder;

    public ShoulderSubsystem(final HardwareMap hMap, String name) {
        shoulder = hMap.get(DcMotorEx.class, name);
        shoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double sup) {
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setPower(sup);
    }

    public void moveToPos(int target, double vel) {
        shoulder.setTargetPosition(target);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setVelocity(vel);
    }

    public boolean isRunningEnc() {
        return shoulder.isBusy();
    }

    public double getEncoderValue() {
        return shoulder.getCurrentPosition();
    }

    public void resetLimit() {
        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
