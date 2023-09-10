package org.firstinspires.ftc.teamcode.Lift;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase {
    private final DcMotorEx lift;

    public LiftSubsystem(final HardwareMap hMap, String name) {
        lift = hMap.get(DcMotorEx.class, name);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double sup, boolean limitEnabled) {
        if (!limitEnabled) {
            lift.setPower(sup);
        } else {
            if (getEncoderValue() >= 4000 || getEncoderValue() <= -10){
                lift.setPower(0);
            } else lift.setPower(sup);
        }
    }

    public double getEncoderValue() {
        return lift.getCurrentPosition();
    }

    public void resetLimit() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
