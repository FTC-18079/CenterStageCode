package org.firstinspires.ftc.teamcode.RRCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

public class TurnCommand extends CommandBase implements Runnable {

    private final SampleMecanumDrive drive;
    private final double angle;

    public TurnCommand(SampleMecanumDrive drive, double angle) {
        this.drive = drive;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        drive.turn(angle);
    }

    @Override
    public void execute() {
        drive.update();
    }

//    @Override
//    public void end(boolean interrupted) {
//        if (interrupted) {
//            drive.stop();
//        }
//    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

    @Override
    public void run() {
        initialize();
    }
}