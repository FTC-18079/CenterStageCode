package org.firstinspires.ftc.teamcode.Chassis;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

import java.util.List;

public class MecanumDrive extends SubsystemBase {
    private final SampleMecanumDrive drive;
    Telemetry m_telemetry;
    private final boolean fieldCentric;
    public PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, boolean isFieldCentric) {
        this.drive = new SampleMecanumDrive(hardwareMap, telemetry);
        m_telemetry = telemetry;
        fieldCentric = isFieldCentric;
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        m_telemetry.addData("Robot Pose Estimate", getPoseEstimate());
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX, double brakePower) {
        double brake = 1.0 - brakePower * 0.8;

        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(
                -leftY * brake,
                -leftX * brake
        ).rotated(fieldCentric ? -poseEstimate.getHeading() : 0);

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightX * brake
                )
        );
        m_telemetry.addData("Brake", brakePower);
    }

    public void collect(double leftY, double leftX, Vector2d targetPos) {
        Pose2d poseEstimate = getPoseEstimate();
        m_telemetry.addData("TARGETING", "true");
        m_telemetry.addData("X", targetPos.getX());
        m_telemetry.addData("Y", targetPos.getY());

        Vector2d input = new Vector2d(
                -leftY,
                -leftX
        );
        Vector2d robotFrameInput = input.rotated(-poseEstimate.getHeading());

        Vector2d difference = targetPos.minus(poseEstimate.vec());
        double theta = difference.angle();
        double thetaFF = -input.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

        headingController.setTargetPosition(theta);

        double headingInput = (headingController.update(poseEstimate.getHeading())
                * DriveConstants.kV + thetaFF)
                * DriveConstants.TRACK_WIDTH;

        Pose2d driveDirection = new Pose2d(robotFrameInput, headingInput);

        drive.setWeightedDrivePower(driveDirection);
        headingController.update(poseEstimate.getHeading());
    }

    public void resetHeading() {
        Pose2d poseEstimate = getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(
                poseEstimate.getX(),
                poseEstimate.getY(),
                0));
    }

    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        drive(0, 0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }
}
