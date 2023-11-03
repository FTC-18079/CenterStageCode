package org.firstinspires.ftc.teamcode.Arm.Lift;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LiftSubsystem extends SubsystemBase {
    private final DcMotorEx lift;
    private final TouchSensor touch;
    private final Telemetry tele;
    private int targetPos;

    public LiftSubsystem(final HardwareMap hMap, String name, String sensor, Telemetry tele) {
        lift = hMap.get(DcMotorEx.class, name);
        touch = hMap.get(TouchSensor.class, sensor);
        this.tele = tele;
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorEx.Direction.REVERSE);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double input, boolean limitEnabled, boolean limitBypass) {
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if (getTouch()) {
            resetLimit();
        }

        if (!limitEnabled || limitBypass) {
            lift.setPower(input);
        } else {
            if (input < 0) {
                if (getEncoderValue() <= Constants.LIFT_LIMIT_TOP) {
                    lift.setPower(0);
                } else lift.setPower(input);
            } else if (input >= 0) {
                if (getEncoderValue() >= Constants.LIFT_LIMIT_BOTTOM || getTouch()){
                    lift.setPower(0);
                } else lift.setPower(input);
            }
        }

        tele.addData("Lift Encoder", getEncoderValue());
    }

    public void stop() {
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveToPos(int target, double vel) {
        double damp;
        targetPos = target;
        lift.setTargetPosition(targetPos);
        if (targetPos > lift.getTargetPosition()) damp = 0.8;
        else damp = 1.0;
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setVelocity(vel * damp);

        tele.addData("Lift Encoder", getEncoderValue());
    }

    public boolean isRunning() {
        return (Math.abs(targetPos - getEncoderValue()) > 5 && lift.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public double getEncoderValue() {
        return lift.getCurrentPosition();
    }

    public boolean getTouch() {
        return touch.isPressed();
    }

    public void resetLimit() {
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        if (!isRunning()) lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
