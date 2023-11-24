package org.firstinspires.ftc.teamcode.Lights;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightSubsystem extends SubsystemBase {
    private final RevBlinkinLedDriver led;

    public LightSubsystem(final HardwareMap hMap, String name) {
        this.led = hMap.get(RevBlinkinLedDriver.class, name);
    }

    public void rainbow() {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    public void redOrange() {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
    }

    public void shot() {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
    }

    public void breath() {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
    }

    public void strobe() {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
    }

    public void heartbeat() {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
    }
}
