package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Test OpMode", group = "Tests")
public class LinearTele extends LinearOpMode {
    private DcMotorEx lift, testMotor, shoulder;
    private Servo axon;
    private RevBlinkinLedDriver led;
    private boolean limitReached = false;
    private boolean limitEnabled = true;

    public DcMotorEx initMotor(String name, Boolean brake, Boolean reverse, Boolean withoutEncoder){
        DcMotorEx motor;
        motor = hardwareMap.get(DcMotorEx.class, name);
        if (brake) motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if (reverse) motor.setDirection(DcMotorEx.Direction.REVERSE);
        else motor.setDirection(DcMotorEx.Direction.FORWARD);

        if (withoutEncoder) motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        else motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        return motor;
    }

    @Override
    public void runOpMode() {

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
            }
            if (gamepad2.b) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
            }
            if (gamepad2.x) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
            }
            if (gamepad2.y) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
            }

        }
    }
}