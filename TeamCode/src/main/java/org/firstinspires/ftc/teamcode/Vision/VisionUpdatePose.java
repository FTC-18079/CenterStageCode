package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Shooter.Chassis.MecanumDrive;

public class VisionUpdatePose extends CommandBase {
    private VisionSubsystem vision;
    private MecanumDrive drive;

    public VisionUpdatePose(VisionSubsystem vision, MecanumDrive drive) {
        this.vision = vision;
        this.drive = drive;
        addRequirements(this.vision);
    }

    @Override
    public void initialize() {
        vision.enableAprilTag();
    }

    @Override
    public void execute() {
        int quadrant = getQuadrant(drive.getPoseEstimate());
        int targetTag = 0;

        if (quadrant == 1) targetTag = 2;
        else if (quadrant == 2) targetTag = 5;
        else if (quadrant == 3) targetTag = 7;
        else if (quadrant == 4) targetTag = 10;

        vision.updatePoseAprilTag(targetTag, drive);
    }

    @Override
    public void end(boolean interrupted) {
        vision.disableAprilTag();
    }

    public int getQuadrant(Pose2d pose) {
        int quadrant = 0;
        boolean xPositive = pose.getX() >= 0;
        boolean yPositive = pose.getY() >= 0;

        if (xPositive && yPositive) return 1;
        else if (xPositive && !yPositive) return 2;
        else if (!xPositive && !yPositive) return 3;
        else if (!xPositive && yPositive) return 4;

        return quadrant;
    }
}
