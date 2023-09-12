package com.maxtech.maxxi.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import static com.maxtech.maxxi.constants.DriveConstants.DEAD_BAND;
import static com.maxtech.maxxi.constants.DriveConstants.SLOW_MODE_FACTOR;

public class HumanDevice {
    private final XboxController playstationStick;

    public HumanDevice(int playstationPort) {
        playstationStick = new XboxController(playstationPort);
    }

    public double getPlaystationY() {
        return modifyAxis(playstationStick.getRawAxis(0) * -1);
    }

    public double getPlaystationX() {
        return modifyAxis(playstationStick.getRawAxis(1) * -1);
    }

    public double getPlaystationR() {
        return modifyAxis(playstationStick.getRawAxis(2) * -1);

    }

    public double getPlaystationTriggerL() {
        return playstationStick.getRawAxis(3);
    }

    public double getPlaystationTriggerR() {
        return playstationStick.getRawAxis(4);
    }

    public JoystickButton setPlaystationCommand(int button) {
        return new JoystickButton(playstationStick, button);
    }

    public POVButton setPlaystationPOV(int angle) {
        return new POVButton(playstationStick, angle);
    }

    private double modifyAxis(double value) {
        value = (Math.abs(value) < DEAD_BAND) ? 0 : value;

        if (playstationStick.getRawButton(5) || playstationStick.getRawButton(11))
            value *= SLOW_MODE_FACTOR;

        return value;
    }
}