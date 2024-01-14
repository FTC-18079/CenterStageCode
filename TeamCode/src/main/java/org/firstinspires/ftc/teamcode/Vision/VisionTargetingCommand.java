package org.firstinspires.ftc.teamcode.Vision;

import com.arcrobotics.ftclib.command.CommandBase;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;

import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class VisionTargetingCommand extends CommandBase {
    private final MecanumDrive drive;
    private final VisionSubsystem vision;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(2.0, 0.0, 0.1);
    private PIDFController pidController;
    private final DoubleSupplier leftY, leftX;
    private final BooleanSupplier slowMode;

    public VisionTargetingCommand(MecanumDrive drive, VisionSubsystem vision, DoubleSupplier leftY, DoubleSupplier leftX, BooleanSupplier slowMode) {
        this.drive = drive;
        this.vision = vision;

        this.leftY = leftY;
        this.leftX = leftX;
        this.slowMode = slowMode;

        addRequirements(this.drive, this.vision);
    }

    @Override
    public void initialize() {
        pidController = new PIDFController(pidCoefficients);
        pidController.setInputBounds(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        double targetAngle = 0.0;
        pidController.setTargetPosition(targetAngle);
        double headingControl = pidController.update(vision.getAprilTagAngle() * Math.PI / 180.0);

//        drive.drive(
//                leftY.getAsDouble(),
//                leftX.getAsDouble(),
//                headingControl,
//                slowMode.getAsBoolean()
//        );
        drive.update();
    }
}
