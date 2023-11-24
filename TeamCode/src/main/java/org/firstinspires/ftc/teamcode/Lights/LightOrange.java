package org.firstinspires.ftc.teamcode.Lights;

import com.arcrobotics.ftclib.command.CommandBase;

public class LightOrange extends CommandBase implements Runnable {
    private final LightSubsystem led;

    public LightOrange(LightSubsystem led) {
        this.led = led;
        addRequirements(this.led);
    }

    @Override
    public void initialize() {
        led.redOrange();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void run() {
        initialize();
    }
}
