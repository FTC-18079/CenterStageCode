package org.firstinspires.ftc.teamcode.Chassis;

import com.arcrobotics.ftclib.command.CommandBase;

public class ResetHeading extends CommandBase {
    private final MecanumDrive drive;

    public ResetHeading(MecanumDrive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetHeading();
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
