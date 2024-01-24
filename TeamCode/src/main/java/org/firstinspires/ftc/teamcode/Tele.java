package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Arm.ArmCommand;
import org.firstinspires.ftc.teamcode.Arm.ArmConstants;
import org.firstinspires.ftc.teamcode.Arm.Lift.StopLift;
import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftCommand;
import org.firstinspires.ftc.teamcode.Arm.Lift.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ResetEncoder;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderCommand;
import org.firstinspires.ftc.teamcode.Arm.Shoulder.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.Chassis.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.Chassis.ResetHeading;
import org.firstinspires.ftc.teamcode.Manip.Claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Manip.Claw.CloseClawTwo;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawOne;
import org.firstinspires.ftc.teamcode.Manip.Claw.MoveClawTwo;
import org.firstinspires.ftc.teamcode.Manip.Stow.Down;
import org.firstinspires.ftc.teamcode.Manip.Stow.Stow;
import org.firstinspires.ftc.teamcode.Manip.Stow.StowSubsystem;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Shooter.FireShooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCommand;
import org.firstinspires.ftc.teamcode.Shooter.ShooterServoCommand;
import org.firstinspires.ftc.teamcode.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Shooter.StopShooter;
import org.firstinspires.ftc.teamcode.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Vision.VisionUpdatePose;

@TeleOp(name = "TeleOp", group = "OpModes")
public class Tele extends CommandOpMode {
    static final double WHEEL_DIAMETER = 96; //millimeters
    //Chassis
    private MotorEx lf, rf, lb, rb;
    private MecanumDrive drive;
    private TeleOpDriveCommand driveCommand;
    private ResetHeading resetHeading;
    //Claw
    private ClawSubsystem claw;
    private MoveClawOne moveClawOne;
    private MoveClawTwo moveClawTwo;
    private CloseClawTwo closeClawTwo;

    //Stow
    private StowSubsystem stow;
    private Stow stowUp;
    private Down stowDown;
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
    private ResetEncoder shoulderReset;
    //Vision
    private VisionSubsystem visionSubsystem;
    private VisionUpdatePose visionUpdatePose;
    private static final String TFOD_MODEL_ASSET = "redObject_v1.tflite";
    private static final String[] LABELS = {
            "redObject"
    };
    private int targetTag;
    //Lights
    private RevBlinkinLedDriver led;

    private Button headingResetButton, liftResetButton, shoulderResetButton, armClimbButton, armMidButton, armLowButton, armRestButton,
            clawOneButton, clawTwoButton, stowButton, shooterButton, blueShooterButton, liftStopButton;

    private Vector2d collectPose = new Vector2d();
    private GamepadEx driverOp, manipOp;

    @Override
    public void initialize() {
        lf = new MotorEx(hardwareMap, "leftFront");
        rf = new MotorEx(hardwareMap, "rightFront");
        lb = new MotorEx(hardwareMap, "leftBack");
        rb = new MotorEx(hardwareMap, "rightBack");
        drive = new MecanumDrive(hardwareMap, telemetry, true);

        // Get pose estimate from auto & determine alliance
        if (PoseStorage.pattern == CP1_SHOT) {
            collectPose = new Vector2d(-55, -55); // Blue Alliance
            targetTag = 7;
        } else {
            collectPose = new Vector2d(-55, 55); // Red Alliance TODO: Change y to 55
            targetTag = 10;
        }
        drive.setPoseEstimate(PoseStorage.currentPose);
        drive.update();

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        shooter = new ShooterSubsystem(hardwareMap, "shooter", "shooterServo");

        lift = new LiftSubsystem(hardwareMap, "lift", "liftTouch", telemetry);
        shoulder = new ShoulderSubsystem(hardwareMap, "shoulder1", "shoulder2", "shoulderTouch", telemetry);

        claw = new ClawSubsystem(hardwareMap, "clawOne", "clawTwo");
        stow = new StowSubsystem(hardwareMap, "stow");

        visionSubsystem = new VisionSubsystem(hardwareMap, "Webcam 1", TFOD_MODEL_ASSET, LABELS, telemetry);
        visionSubsystem.enableAprilTag();

//        m_telemetry = new TelemetrySS(telemetry);

        driverOp = new GamepadEx(gamepad1);
        manipOp = new GamepadEx(gamepad2);

        driveCommand = new TeleOpDriveCommand(
                drive,
                () -> -driverOp.getLeftY(),
                () -> driverOp.getLeftX(),
                () -> driverOp.getRightX(),
                () -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                collectPose,
                () -> driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER),
                () -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
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
        shoulderReset = new ResetEncoder(shoulder);

        stopShooter = new StopShooter(shooter);
        fireServo = new ShooterServoCommand(shooter,0);

        moveClawOne = new MoveClawOne(claw);
        moveClawTwo = new MoveClawTwo(claw);
        closeClawTwo = new CloseClawTwo(claw);
        stowUp = new Stow(stow);
        stowDown = new Down(stow);

        visionUpdatePose = new VisionUpdatePose(visionSubsystem, drive, () -> targetTag);

        headingResetButton = (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whenPressed(resetHeading);
        shooterButton = (new GamepadButton(driverOp, GamepadKeys.Button.B))
                .whenReleased(new ShooterCommand(shooter, () -> 0.55, telemetry),true);

        clawOneButton = (new GamepadButton(manipOp, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(moveClawOne, true);
        clawTwoButton = (new GamepadButton(manipOp, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(moveClawTwo, true);
        shoulderResetButton = (new GamepadButton(manipOp, GamepadKeys.Button.A))
                .whenPressed(shoulderReset);
        stowButton = (new GamepadButton(manipOp, GamepadKeys.Button.B))
                .whenPressed(stowUp, true)
                .whenReleased(stowDown, true);

        liftStopButton = (new GamepadButton(manipOp, GamepadKeys.Button.Y))
                .whenPressed(stopLift, true);
        armClimbButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_LEFT))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> ArmConstants.SHOULDER_POS_CLIMB, () -> ArmConstants.LIFT_POS_CLIMB, () -> ArmConstants.STOW_POS_CLIMB, telemetry), true);
        armLowButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_RIGHT))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> ArmConstants.SHOULDER_POS_LOW, () -> ArmConstants.LIFT_POS_LOW, () -> ArmConstants.STOW_POS_LOW, telemetry), true);
        armMidButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_UP))
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> ArmConstants.SHOULDER_POS_MID, () -> ArmConstants.LIFT_POS_MID, () -> ArmConstants.STOW_POS_MID, telemetry), true);
        armRestButton = (new GamepadButton(manipOp, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(closeClawTwo, true)
                .whenReleased(new ArmCommand(shoulder, lift, stow,
                        () -> ArmConstants.SHOULDER_POS_REST, () -> ArmConstants.LIFT_POS_REST, () -> ArmConstants.STOW_POS_REST, telemetry), true);

        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

//        register(visionSubsystem);
        register(drive);
        register(lift);
        register(shoulder);
//        visionSubsystem.setDefaultCommand(visionUpdatePose);
        drive.setDefaultCommand(driveCommand);
        lift.setDefaultCommand(liftCommand);
        shoulder.setDefaultCommand(shoulderCommand);
        led.setPattern(PoseStorage.pattern);
    }
}