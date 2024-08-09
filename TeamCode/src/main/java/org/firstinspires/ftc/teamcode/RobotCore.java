package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Arm.ArmCommand;
import org.firstinspires.ftc.teamcode.Arm.ArmConstants;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftToPos;
import org.firstinspires.ftc.teamcode.Arm.Lift.StopLift;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderToPos;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.Chassis.ResetHeading;
import org.firstinspires.ftc.teamcode.Chassis.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCommand;
import org.firstinspires.ftc.teamcode.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.legacyclaw.LegacyClaw;

public class RobotCore extends Robot {
    Telemetry telemetry;
    GamepadEx driveController;
    // Subsystems
    MecanumDrive chassis;
    LiftSubsystem lift;
    ShoulderSubsystem pivot;
    ShooterSubsystem shooter;
    LegacyClaw claw;
    VisionSubsystem vision;
    // Commands
    TeleOpDriveCommand driveCommand;

    public RobotCore(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Pose2d initialPose) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chassis = new MecanumDrive(hardwareMap, telemetry, true);
        lift = new LiftSubsystem(hardwareMap, "lift", "liftTouch", telemetry);
        pivot = new ShoulderSubsystem(hardwareMap, "shoulder1", "shoulder2", "shoulderTouch", telemetry);
        shooter = new ShooterSubsystem(hardwareMap, "shooter", "shooterServo");
        claw = new LegacyClaw(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, "Webcam 1", "redObject_v1.tflite", new String[] {"redObject"}, telemetry);

        register(chassis);
        register(lift);
        register(pivot);
        register(shooter);
        register(claw);
        register(vision);

        setDriveControls();
    }

    private void setDriveControls() {
        driveCommand = new TeleOpDriveCommand(
                chassis,
                () -> -driveController.getLeftY() * 0.5,
                () -> driveController.getLeftX() * 0.5,
                () -> driveController.getRightX() * 0.5,
                vision
        );
        driveController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ResetHeading(chassis));

        new Trigger(() -> driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2)
                .whenActive(new ShooterCommand(shooter, () -> 0.55, telemetry),true);

        driveController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(claw::stowUp),
                        new WaitCommand(20),
                        new InstantCommand(claw::rotate),
                        new WaitCommand(100),
                        new InstantCommand(claw::stowDown)
                ));

        driveController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(claw::grab));
        driveController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(claw::release));

        driveController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new StopLift(lift), true);

        driveController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ShoulderToPos(pivot, () -> ArmConstants.SHOULDER_POS_CLIMB, () -> ArmConstants.SHOULDER_VELOCITY, telemetry), true)
                .whenPressed(new LiftToPos(lift, () -> ArmConstants.LIFT_POS_CLIMB, () -> ArmConstants.LIFT_VELOCITY, telemetry), true);
        driveController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ShoulderToPos(pivot, () -> ArmConstants.SHOULDER_POS_LOW, () -> ArmConstants.SHOULDER_VELOCITY, telemetry), true)
                .whenPressed(new LiftToPos(lift, () -> ArmConstants.LIFT_POS_LOW, () -> ArmConstants.LIFT_VELOCITY, telemetry), true);
        driveController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ShoulderToPos(pivot, () -> ArmConstants.SHOULDER_POS_MID, () -> ArmConstants.SHOULDER_VELOCITY, telemetry), true)
                .whenPressed(new LiftToPos(lift, () -> ArmConstants.LIFT_POS_MID, () -> ArmConstants.LIFT_VELOCITY, telemetry), true);
        driveController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ShoulderToPos(pivot, () -> ArmConstants.SHOULDER_POS_REST, () -> ArmConstants.SHOULDER_VELOCITY, telemetry), true)
                .whenPressed(new LiftToPos(lift, () -> ArmConstants.LIFT_POS_REST, () -> ArmConstants.LIFT_VELOCITY, telemetry), true);
    }

    public double responseCurve(double value, double power) {
//        value = deadzone(value, DEADZONE);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        this.telemetry.update();
    }
}
