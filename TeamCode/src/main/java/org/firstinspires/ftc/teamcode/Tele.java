package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Arm.ArmCommand;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderPos;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftCommand;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Lift.ResetLimit;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ResetEncoder;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderCommand;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Claw.AutoMoveClaw;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Manip.WristStow;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryCommand;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetrySS;
import org.firstinspires.ftc.teamcode.Manip.Wrist.WristCommand;
import org.firstinspires.ftc.teamcode.Manip.Wrist.WristSubsystem;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Tele extends CommandOpMode {
    static final double WHEEL_DIAMETER = 96; //millimeters
//Chassis
    private MotorEx lf, rf, lb, rb;
    private MecanumDrive drive;
    private MecanumDriveCommand driveCommand;
//Claw
    private ClawSubsystem claw;
//Wrist
    private WristSubsystem wrist;
    private WristCommand wristCommand;
//Stow
    private StowSubsystem stow;
//Lift
    private LiftSubsystem lift;
    private LiftCommand liftCommand;
    private ResetLimit liftReset;
//Shoulder
    private ShoulderSubsystem shoulder;
    private ShoulderCommand shoulderCommand;
    private ShoulderPos getShoulderPos;
    private ResetEncoder shoulderReset;

    private Button liftResetButton, shoulderResetButton, armUpButton, armMidButton, armLowButton, armRestButton,
            clawButton, wristButton, stowButton;
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
        shoulder = new ShoulderSubsystem(hardwareMap, "shoulder1", "shoulder2");

        claw = new ClawSubsystem(hardwareMap,"clawOne", "clawTwo");
        wrist = new WristSubsystem(hardwareMap, "wrist", "wristEncoder");
        stow = new StowSubsystem(hardwareMap, "stow");

        m_telemetry = new TelemetrySS(telemetry);

        driverOp = new GamepadEx(gamepad1);
        manipOp = new GamepadEx(gamepad2);

        driveCommand = new MecanumDriveCommand(
                drive,
                () -> -driverOp.getLeftY() * 0.8,
                () -> driverOp.getLeftX() * 0.8,
                () -> driverOp.getRightX() * 0.8
        );

        liftCommand = new LiftCommand(
                lift,
                () -> manipOp.getRightY() * 0.5,
                () -> true,
                () -> manipOp.getButton(GamepadKeys.Button.X)
        );
        liftReset = new ResetLimit(lift);

        shoulderCommand = new ShoulderCommand(
                shoulder,
                () -> manipOp.getLeftY() * 0.5
        );
        getShoulderPos = new ShoulderPos(shoulder);
        shoulderReset = new ResetEncoder(shoulder);

        wristCommand = new WristCommand(wrist);

        telemetryCommand = new TelemetryCommand(
                m_telemetry,
                () -> lift.getEncoderValue(),
                () -> shoulder.getEncoderValue(),
                () -> wrist.getPos(),
                () -> wrist.getEncoderPos(),
                () -> claw.getClawOnePos(),
                () -> claw.getClawTwoPos()
        );

        liftResetButton = (new GamepadButton(manipOp, GamepadKeys.Button.Y))
                .whenPressed(liftReset);
        shoulderResetButton = (new GamepadButton(manipOp, GamepadKeys.Button.A))
                .whenPressed(shoulderReset);

        clawButton = (new GamepadButton(manipOp, GamepadKeys.Button.B))
                .whenReleased(new AutoMoveClaw(claw, wrist, shoulder));
        wristButton = (new GamepadButton(manipOp, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(new WristStow(wrist, stow, shoulder))
                .whenReleased(new Down(stow));

        armUpButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_UP))
                .whenPressed(new ArmCommand(shoulder, lift, () -> Constants.SHOULDER_POS_HIGH, () -> Constants.LIFT_POS_HIGH));
        armMidButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_RIGHT))
                .whenPressed(new ArmCommand(shoulder, lift, () -> Constants.SHOULDER_POS_MID, () -> Constants.LIFT_POS_MID));
        armLowButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(new ArmCommand(shoulder, lift, () -> Constants.SHOULDER_POS_LOW, () -> Constants.LIFT_POS_LOW));
        armRestButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_LEFT))
                .whenPressed(new ArmCommand(shoulder, lift, () -> Constants.SHOULDER_POS_REST, () -> Constants.LIFT_POS_REST));

        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        register(drive);
        register(lift);
        register(shoulder);
        register(m_telemetry);
        drive.setDefaultCommand(driveCommand);
        lift.setDefaultCommand(liftCommand);
        shoulder.setDefaultCommand(shoulderCommand);
        m_telemetry.setDefaultCommand(telemetryCommand);
    }
}