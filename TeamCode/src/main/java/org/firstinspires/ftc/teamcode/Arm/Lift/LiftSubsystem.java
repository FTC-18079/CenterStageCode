package org.firstinspires.ftc.teamcode.Arm.Lift;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class LiftSubsystem extends SubsystemBase {
    private final DcMotorEx lift;

    public LiftSubsystem(final HardwareMap hMap, String name) {
        lift = hMap.get(DcMotorEx.class, name);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double sup, boolean limitEnabled, boolean limitBypass) {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!limitEnabled || limitBypass) {
            lift.setPower(sup);
        } else {
            if (getEncoderValue() <= Constants.LIFT_LIMIT_TOP || getEncoderValue() >= Constants.LIFT_LIMIT_BOTTOM){
                lift.setPower(0);
            } else lift.setPower(sup);
        }
    }

    public void moveToPos(int target, double vel) {
        lift.setTargetPosition(target);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setVelocity(vel);
    }

    public boolean isRunningEnc() {
        return lift.isBusy();
    }

    public double getEncoderValue() {
        return lift.getCurrentPosition();
    }

    public void resetLimit() {
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
