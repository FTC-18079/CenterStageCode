package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Chassis.DriveToCollect;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Chassis.ResetHeading;

@TeleOp(name = "FUNZIES", group = "TEST")
public class TeleFun extends CommandOpMode {
    //Chassis
    private MotorEx lf, rf, lb, rb;
    private MecanumDrive drive;
    private MecanumDriveCommand driveCommand;
    private ResetHeading resetHeading;
    private DriveToCollect driveToCollect;

    private GamepadEx driverOp;
    private Button headingResetButton, funButton;
    private Vector2d targetPos = new Vector2d(0, 0);

    @Override
    public void initialize() {
        lf = new MotorEx(hardwareMap, "leftFront");
        rf = new MotorEx(hardwareMap, "rightFront");
        lb = new MotorEx(hardwareMap, "leftBack");
        rb = new MotorEx(hardwareMap, "rightBack");
        drive = new MecanumDrive(hardwareMap, telemetry, true);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        drive.update();

        driverOp = new GamepadEx(gamepad1);

        driveCommand = new MecanumDriveCommand(
                drive,
                () -> -driverOp.getLeftY(),
                () -> driverOp.getLeftX(),
                () -> driverOp.getRightX(),
                () -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        );
        resetHeading = new ResetHeading(drive);

        headingResetButton = (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whenReleased(resetHeading);
        funButton = (new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(new DriveToCollect(
                        drive,
                        () -> -driverOp.getLeftY(),
                        () -> driverOp.getLeftX(),
                        targetPos
                ));

        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        register(drive);
        drive.setDefaultCommand(driveCommand);
    }
}
