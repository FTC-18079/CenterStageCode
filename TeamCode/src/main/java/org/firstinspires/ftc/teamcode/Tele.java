package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.Chassis.ResetHeading;
import org.firstinspires.ftc.teamcode.Manip.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Claw.AutoMoveClaw;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.Stow;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowCommand;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Manip.WristStow;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.StandardTrackingWheelLocalizer;
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
    private ResetHeading resetHeading;
    //Claw
    private ClawSubsystem claw;
    //Wrist
    private WristSubsystem wrist;
    private WristCommand wristCommand;
    //Stow
    private StowSubsystem stow;
    private Stow stowUp;
    private Down stowDown;
    private StowCommand moveStow;
    //Lift
    private LiftSubsystem lift;
    private LiftCommand liftCommand;
    private ResetLimit liftReset;
    //Shoulder
    private ShoulderSubsystem shoulder;
    private ShoulderCommand shoulderCommand;
    private ShoulderPos getShoulderPos;
    private ResetEncoder shoulderReset;

    private Button headingResetButton, liftResetButton, shoulderResetButton, armUpButton, armMidButton, armLowButton, armRestButton,
            clawButton, wristButton, stowButton;
    private GamepadEx driverOp, manipOp;
    //    private TelemetrySS m_telemetry;
    private TelemetryCommand telemetryCommand;

    @Override
    public void initialize() {
        lf = new MotorEx(hardwareMap, "leftFront");
        rf = new MotorEx(hardwareMap, "rightFront");
        lb = new MotorEx(hardwareMap, "leftBack");
        rb = new MotorEx(hardwareMap, "rightBack");
        drive = new MecanumDrive(hardwareMap, telemetry, true);
        if (PoseStorage.hasAutoRun) drive.setPoseEstimate(PoseStorage.currentPose);
        else drive.setPoseEstimate(new Pose2d());

        lift = new LiftSubsystem(hardwareMap, "lift", "liftTouch", telemetry);
        shoulder = new ShoulderSubsystem(hardwareMap, "shoulder1", "shoulder2", "shoulderTouch", telemetry);

        claw = new ClawSubsystem(hardwareMap, "clawOne", "clawTwo");
        wrist = new WristSubsystem(hardwareMap, "wrist");
        stow = new StowSubsystem(hardwareMap, "stow");

//        m_telemetry = new TelemetrySS(telemetry);

        driverOp = new GamepadEx(gamepad1);
        manipOp = new GamepadEx(gamepad2);

        driveCommand = new MecanumDriveCommand(
                drive,
                () -> -driverOp.getLeftY() * 0.85,
                () -> driverOp.getLeftX() * 0.85,
                () -> driverOp.getRightX() * 0.85,
                () -> driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)
        );
        resetHeading = new ResetHeading(drive);

        liftCommand = new LiftCommand(
                lift,
                () -> manipOp.getRightY() * 0.5,
                () -> true,
                () -> manipOp.getButton(GamepadKeys.Button.X)
        );

        shoulderCommand = new ShoulderCommand(
                shoulder,
                () -> manipOp.getLeftY() * 0.5
        );
        getShoulderPos = new ShoulderPos(shoulder);
        shoulderReset = new ResetEncoder(shoulder);

        wristCommand = new WristCommand(wrist);

        stowUp = new Stow(stow);
        stowDown = new Down(stow);
        moveStow = new StowCommand(stow);

        headingResetButton = (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whenReleased(resetHeading);

//        liftResetButton = (new GamepadButton(manipOp, GamepadKeys.Button.Y))
//                .whenPressed(liftReset);
        shoulderResetButton = (new GamepadButton(manipOp, GamepadKeys.Button.A))
                .whenPressed(shoulderReset);

        stowButton = (new GamepadButton(manipOp, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(stowUp, true)
                .whenReleased(stowDown, true);
        clawButton = (new GamepadButton(manipOp, GamepadKeys.Button.B))
                .whenReleased(new AutoMoveClaw(claw, wrist, shoulder), true);
        wristButton = (new GamepadButton(manipOp, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(new WristStow(wrist, stow, shoulder.getEncoderValue()))
                .whenReleased(stowDown, true);

        armUpButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_UP))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> Constants.SHOULDER_POS_HIGH, () -> Constants.LIFT_POS_HIGH, () -> Constants.STOW_POS_HIGH, telemetry), true);
        armMidButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_RIGHT))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> Constants.SHOULDER_POS_MID, () -> Constants.LIFT_POS_MID, () -> Constants.STOW_POS_MID, telemetry), true);
        armLowButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_DOWN))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> Constants.SHOULDER_POS_LOW, () -> Constants.LIFT_POS_LOW, () -> Constants.STOW_POS_LOW, telemetry), true);
        armRestButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_LEFT))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> Constants.SHOULDER_POS_REST, () -> Constants.LIFT_POS_REST, () -> Constants.STOW_POS_REST, telemetry), true);

        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        register(drive);
        register(lift);
        register(shoulder);
//        register(m_telemetry);
        drive.setDefaultCommand(driveCommand);
        lift.setDefaultCommand(liftCommand);
        shoulder.setDefaultCommand(shoulderCommand);
//        m_telemetry.setDefaultCommand(telemetryCommand);
    }
}