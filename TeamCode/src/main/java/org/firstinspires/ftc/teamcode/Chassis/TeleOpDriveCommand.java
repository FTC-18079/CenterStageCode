package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Vision.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrive drive;

    private final DoubleSupplier leftY, leftX, rightX, brakePower;
    private final BooleanSupplier collecting, dumping;
    private final DoubleSupplier rightTrigger;
    private final Vector2d targetPos;
    private final VisionSubsystem vision;

    public TeleOpDriveCommand(MecanumDrive drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, DoubleSupplier brakePower, Vector2d targetPos, BooleanSupplier collecting, DoubleSupplier rightTrigger, BooleanSupplier dumping, VisionSubsystem vision) {
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        this.brakePower = brakePower;
        this.targetPos = targetPos;
        this.collecting = () -> false; // collecting;
        this.rightTrigger = rightTrigger;
        this.dumping = dumping;
        this.vision = vision;
        addRequirements(drive);
    }

    public TeleOpDriveCommand(MecanumDrive drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, VisionSubsystem vision) {
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        brakePower = () -> 0;
        targetPos = new Vector2d();
        collecting = () -> false;
        dumping = () -> false;
        rightTrigger = () -> 0;
        this.vision = vision;
    }

    @Override
    public void execute() {
        AprilTagDetection tag = vision.getAprilTagDetection(PoseStorage.dumpingTag);
        drive.driveCollect(
                leftY.getAsDouble(),
                leftX.getAsDouble(),
                rightX.getAsDouble(),
                brakePower.getAsDouble(),
                targetPos,
                collecting.getAsBoolean(),
                rightTrigger.getAsDouble(),
                dumping.getAsBoolean(),
                tag
        );
        drive.update();
    }
}
