package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Vision.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.IntSupplier;

@Config
public class AutoDump extends CommandBase {
    private final MecanumDrive drive;
    private final VisionSubsystem vision;
    private final IntSupplier tagId;
    public static double DESIRED_DISTANCE = 7.5;

    public AutoDump(MecanumDrive drive, VisionSubsystem vision, IntSupplier tagId) {
        this.drive = drive;
        this.vision = vision;
        this.tagId = tagId;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        AprilTagDetection tag = vision.getAprilTagDetection(tagId.getAsInt());
        if (tag != null) {
            double rangeError = (tag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = tag.ftcPose.bearing;
            double yawError = tag.ftcPose.yaw;

            double fwd  = Range.clip(rangeError * 0.02, -0.5, 0.5);
            double turn   = Range.clip(headingError * 0.01, -0.3, 0.3) ;
            double strafe = Range.clip(-yawError * 0.015, -0.5, 0.5);

             drive.driveCollect(fwd, strafe, turn, 0, new Vector2d(), false, 0);
        } else drive.drive(new Pose2d());
        drive.update();
    }
}
