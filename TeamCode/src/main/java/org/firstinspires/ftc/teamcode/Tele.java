package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Arm.ArmCommand;
import org.firstinspires.ftc.teamcode.Arm.Lift.StopLift;
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
import org.firstinspires.ftc.teamcode.Manip.Claw.CloseClawOne;
import org.firstinspires.ftc.teamcode.Manip.Claw.CloseClawTwo;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawOne;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawTwo;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.Stow;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowCommand;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Shooter.FireShooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCommand;
import org.firstinspires.ftc.teamcode.Shooter.ShooterServoCommand;
import org.firstinspires.ftc.teamcode.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Shooter.StopShooter;
import org.firstinspires.ftc.teamcode.Telemetry.TelemetryCommand;

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
    private MoveClawOne moveClawOne;
    private MoveClawTwo moveClawTwo;
    private CloseClawOne closeClawOne;
    private CloseClawTwo closeClawTwo;

    //Stow
    private StowSubsystem stow;
    private Stow stowUp;
    private Down stowDown;
    private StowCommand moveStow;
    //Shooter
    private ShooterSubsystem shooter;
    private FireShooter fireShooter;
    private StopShooter stopShooter;
    private ShooterServoCommand fireServo;
    //Lift
    private LiftSubsystem lift;
    private LiftCommand liftCommand;
    private StopLift stopLift;
    //Shoulder
    private ShoulderSubsystem shoulder;
    private ShoulderCommand shoulderCommand;
    private ShoulderPos getShoulderPos;
    private ResetEncoder shoulderReset;

    private Button headingResetButton, liftResetButton, shoulderResetButton, armClimbButton, armMidButton, armLowButton, armRestButton,
            clawOneButton, clawTwoButton, stowButton, shooterCommandButton, shooterButton, liftStopButton, fireShooterButton;
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

        if (PoseStorage.hasAutoRun) drive.setPoseEstimate(PoseStorage.currentPose.plus(new Pose2d(0, 0, Math.toRadians(180))));
        else drive.setPoseEstimate(new Pose2d());

        shooter = new ShooterSubsystem(hardwareMap, "shooter", "shooterServo");

        lift = new LiftSubsystem(hardwareMap, "lift", "liftTouch", telemetry);
        shoulder = new ShoulderSubsystem(hardwareMap, "shoulder1", "shoulder2", "shoulderTouch", telemetry);

        claw = new ClawSubsystem(hardwareMap, "clawOne", "clawTwo");
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
                () -> manipOp.getRightY(),
                () -> true,
                () -> manipOp.getButton(GamepadKeys.Button.X)
        );
        stopLift = new StopLift(lift);

        shoulderCommand = new ShoulderCommand(
                shoulder,
                () -> manipOp.getLeftY() * 0.5
        );
        getShoulderPos = new ShoulderPos(shoulder);
        shoulderReset = new ResetEncoder(shoulder);

        fireShooter = new FireShooter(shooter);
        stopShooter = new StopShooter(shooter);
        fireServo = new ShooterServoCommand(shooter,0);

        moveClawOne = new MoveClawOne(claw);
        moveClawTwo = new MoveClawTwo(claw);
        closeClawOne = new CloseClawOne(claw);
        closeClawTwo = new CloseClawTwo(claw);
        stowUp = new Stow(stow);
        stowDown = new Down(stow);
        moveStow = new StowCommand(stow);

        headingResetButton = (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whenReleased(resetHeading);
        fireShooterButton = (new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(fireServo, true);

        clawOneButton = (new GamepadButton(manipOp, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(moveClawOne, true);
        clawTwoButton = (new GamepadButton(manipOp, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(moveClawTwo, true);
        shoulderResetButton = (new GamepadButton(manipOp, GamepadKeys.Button.A))
                .whenPressed(shoulderReset);
        stowButton = (new GamepadButton(manipOp, GamepadKeys.Button.B))
                .whenPressed(stowUp, true)
                .whenReleased(stowDown, true);
        shooterButton = (new GamepadButton(driverOp, GamepadKeys.Button.B))
                .whenPressed(fireShooter, true)
                .whenReleased(stopShooter, true);
        shooterCommandButton = (new GamepadButton(driverOp, GamepadKeys.Button.X)).
                whenReleased(new ShooterCommand(shooter, telemetry), true);

        liftStopButton = (new GamepadButton(manipOp, GamepadKeys.Button.Y))
                .whenPressed(stopLift, true);
        armClimbButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_LEFT))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> Constants.SHOULDER_POS_CLIMB, () -> Constants.LIFT_POS_CLIMB, () -> Constants.STOW_POS_CLIMB, telemetry), true);
        armMidButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_RIGHT))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> Constants.SHOULDER_POS_MID, () -> Constants.LIFT_POS_MID, () -> Constants.STOW_POS_MID, telemetry), true);
        armLowButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_UP))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> Constants.SHOULDER_POS_LOW, () -> Constants.LIFT_POS_LOW, () -> Constants.STOW_POS_LOW, telemetry), true);
        armRestButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(closeClawTwo, true)
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