package com.maxtech.maxxi.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import static com.maxtech.maxxi.constants.DriveConstants.DEAD_BAND;

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
        return modifyAxis(driverStick.getY()) * -1;
    }
    public double getDriverY() {
        return modifyAxis(driverStick.getX()) * -1;
    }

    public double getDriverT() {
        return modifyAxis(driverStick.getTwist()) * -1;
    }

    public double getOperatorX() {
        SmartDashboard.putNumber("Operator X " , modifyAxis(operatorStick.getLeftY() * -1));
        return modifyAxis(operatorStick.getLeftY() * -1);
    }

    public double getOperatorY() {
        SmartDashboard.putNumber("Operator Y " , modifyAxis(operatorStick.getLeftX() * -1));
        return modifyAxis(operatorStick.getLeftX() * -1);
    }

    public double getOperatorR() {
        SmartDashboard.putNumber("Operator R " , modifyAxis(operatorStick.getRightX() * -1));
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
        value = Math.copySign(value * value, value);
        if (driverStick.getRawButton(1)) {
            value = Math.copySign(value * 0.5, value);
        }
        value *= 0.4;
        return value;
    }
}