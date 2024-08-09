package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LIBRARY TELE")
public class TeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotCore robot = new RobotCore(hardwareMap, telemetry, gamepad1, new Pose2d());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }

        robot.reset();
    }
}
