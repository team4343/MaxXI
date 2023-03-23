package com.maxtech.maxxi.util;

import static com.maxtech.maxxi.constants.DriveConstants.DEAD_BAND;
import com.maxtech.maxxi.constants.DriveConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class HumanDevice {
    private final Joystick driverStick;
    private final XboxController operatorStick;
    public static int alliance_modifier = 1;


    public HumanDevice(int driverPort, int operatorPort) {
        driverStick = new Joystick(driverPort);
        operatorStick = new XboxController(operatorPort);
    }

    // Joystick X and Y are swapped due to orientation of the joystick relative to the field.
    public double getDriverX() {
        return modifyAxis(driverStick.getY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * -1;
    }
    public double getDriverY() {
        return modifyAxis(driverStick.getX()) * -1;
    }

    public double getDriverT() {
        return modifyAxis(driverStick.getTwist()) * -1;
    }

    public double getOperatorX() {
        return modifyAxis(operatorStick.getLeftY() * -1);
    }

    public double getOperatorY() {
        return modifyAxis(operatorStick.getLeftX() * -1);
    }

    public double getOperatorR() {
        return modifyAxis(operatorStick.getRightX() * -1);
    }

    public double getOperatorTriggerL() {
        return operatorStick.getLeftTriggerAxis();
    }

    public double getOperatorTriggerR() {
        return operatorStick.getRightTriggerAxis();
    }

    public JoystickButton setDriverCommand(int button) {
        return new JoystickButton(driverStick, button);
    }

    public JoystickButton setOperatorCommand(int button) {
        return new JoystickButton(operatorStick, button);
    }

    public POVButton setDriverPOV(int angle) {
        return new POVButton(driverStick, angle);
    }

    public POVButton setOperatorPOV(int angle) {
        return new POVButton(operatorStick, angle);
    }


    private double modifyAxis(double value) {
        value = (Math.abs(value) < DEAD_BAND) ? 0 : value;
        value = Math.copySign(value * value, value); // TODO: Make cubed?

        return value;
    }
}