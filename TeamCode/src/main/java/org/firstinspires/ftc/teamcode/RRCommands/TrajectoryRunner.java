package org.firstinspires.ftc.teamcode.RRCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Chassis.MecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectoryRunner extends CommandBase implements Runnable {
    private final MecanumDrive drive;
    private final TrajectorySequence traj;

    public TrajectoryRunner(MecanumDrive drive, TrajectorySequence traj) {
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(traj);
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

    @Override
    public void run() {
        initialize();
    }
}
