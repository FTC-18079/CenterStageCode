package org.firstinspires.ftc.teamcode.Vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;

import java.util.function.IntSupplier;

public class VisionUpdatePose extends CommandBase {
    private VisionSubsystem vision;
    private MecanumDrive drive;
    private IntSupplier targetTag;

    public VisionUpdatePose(VisionSubsystem vision, MecanumDrive drive, IntSupplier targetTag) {
        this.vision = vision;
        this.drive = drive;
        this.targetTag = targetTag;
        addRequirements(this.vision);
    }

    @Override
    public void initialize() {
        vision.enableAprilTag();
    }

    @Override
    public void execute() {
        vision.updatePoseAprilTag(targetTag.getAsInt(), drive);
    }

    @Override
    public void end(boolean interrupted) {
        vision.disableAprilTag();
    }
}
