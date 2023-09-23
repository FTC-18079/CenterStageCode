package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Lift.LiftCommand;
import org.firstinspires.ftc.teamcode.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Lift.ResetLimit;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryCommand;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetrySS;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Tele extends CommandOpMode {
    static final double WHEEL_DIAMETER = 96; //millimeters

    private MotorEx lf, rf, lb, rb;
    private MecanumDrive drive;
    private MecanumDriveCommand driveCommand;
    private LiftSubsystem lift;
    private LiftCommand liftCommand;
    private ResetLimit liftReset;

    private Button liftResetButton;
    private GamepadEx driverOp, manipOp;
    private TelemetrySS m_telemetry;
    private TelemetryCommand telemetryCommand;

    @Override
    public void initialize() {
        lf = new MotorEx(hardwareMap, "leftFront");
        rf = new MotorEx(hardwareMap, "rightFront");
        lb = new MotorEx(hardwareMap, "leftBack");
        rb = new MotorEx(hardwareMap, "rightBack");
        drive = new MecanumDrive(hardwareMap, telemetry, false);

        lift = new LiftSubsystem(hardwareMap, "lift");

        m_telemetry = new TelemetrySS(telemetry);

        driverOp = new GamepadEx(gamepad1);
        manipOp = new GamepadEx(gamepad2);

        driveCommand = new MecanumDriveCommand(
                drive,
                () -> -driverOp.getLeftY(),
                () -> driverOp.getLeftX(),
                () -> driverOp.getRightX()
        );

        liftCommand = new LiftCommand(
                lift,
                () -> -manipOp.getLeftY(),
                () -> true,
                () -> manipOp.getButton(GamepadKeys.Button.X)
        );
        liftReset = new ResetLimit(lift);

        telemetryCommand = new TelemetryCommand(
                m_telemetry,
                () -> lift.getEncoderValue()
        );

        liftResetButton = (new GamepadButton(manipOp, GamepadKeys.Button.Y))
                .whenPressed(liftReset);

        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        register(drive);
        register(lift);
        register(m_telemetry);
        drive.setDefaultCommand(driveCommand);
        lift.setDefaultCommand(liftCommand);
        m_telemetry.setDefaultCommand(telemetryCommand);
    }
}
