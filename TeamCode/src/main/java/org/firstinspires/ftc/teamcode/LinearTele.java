package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test OpMode", group = "Tests")
public class LinearTele extends LinearOpMode {
    private DcMotorEx lift, testMotor;
    private Servo axon;
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
        lift = initMotor("lift", true, true, true);
        testMotor = initMotor("wristEncoder", false, false, true);
        testMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        axon = hardwareMap.get(Servo.class, "wrist");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!limitEnabled) {
                limitReached = false;
            } else {
                limitReached = (lift.getCurrentPosition() >= 4000) || (lift.getCurrentPosition() <= -10);
                if (gamepad1.x) {
                    limitEnabled = false;
                }
            }

            if (gamepad1.y) {
                limitEnabled = true;
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (!limitReached) {
                lift.setPower(-gamepad1.left_stick_y * 1);
            } else lift.setPower(0);

            if(gamepad1.a){
                axon.setPosition(0);
            }
            if(gamepad1.b){
                axon.setPosition(1);
            }

            telemetry.addData("Motor Pos", lift.getCurrentPosition());
            telemetry.addData("Motor Power", -gamepad1.left_stick_y * 100);
            telemetry.addData("Axon encoder", testMotor.getCurrentPosition());
            telemetry.addData("Servo", axon.getPosition());
            telemetry.update();
        }
    }
}
