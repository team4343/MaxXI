package com.maxtech.maxxi.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import static com.maxtech.maxxi.constants.DriveConstants.DEAD_BAND;
import static com.maxtech.maxxi.constants.DriveConstants.SLOW_MODE_FACTOR;

public class HumanDevice {
    private final Joystick driverStick;
    private final XboxController operatorStick;
    private final XboxController playstationStick;

    public HumanDevice(int driverPort, int operatorPort, int playstationPort) {
        driverStick = new Joystick(driverPort);
        operatorStick = new XboxController(operatorPort);
        playstationStick = new XboxController(playstationPort);
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

    public JoystickButton setDriverCommand(int button) {
        return new JoystickButton(driverStick, button);
    }

    public JoystickButton setOperatorCommand(int button) {
        return new JoystickButton(operatorStick, button);
    }

    public JoystickButton setPlaystationCommand(int button) {
        return new JoystickButton(playstationStick, button);
    }

    public POVButton setDriverPOV(int angle) {
        return new POVButton(driverStick, angle);
    }

    public POVButton setOperatorPOV(int angle) {
        return new POVButton(operatorStick, angle);
    }

    public POVButton setPlaystationPOV(int angle) {
        return new POVButton(playstationStick, angle);
    }


    private double modifyAxis(double value) {
        value = (Math.abs(value) < DEAD_BAND) ? 0 : value;

        if (playstationStick.getRawButton(5) || driverStick.getRawButton(1))
            value *= SLOW_MODE_FACTOR;

        return value;
    }
}