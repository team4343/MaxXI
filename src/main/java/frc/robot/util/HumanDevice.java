package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.DriveConstants;

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
        return modifyAxis(driverStick.getX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }
    public double getDriverY() {
        return modifyAxis(driverStick.getY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * -1;
    }

    public double getDriverT() {
        return modifyAxis(driverStick.getTwist()) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    public double getOperatorLeftX() {
        return modifyAxis(operatorStick.getLeftY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * -1;
    }

    public double getOperatorLeftY() {
        return modifyAxis(operatorStick.getLeftX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }

    public double getOperatorRightX() {
        return modifyAxis(operatorStick.getRightY()) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * -1;
    }

    public double getOperatorRightY() {
        return modifyAxis(operatorStick.getRightX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * -1;
    }

    public JoystickButton setDriverCommand(int button) {
        return new JoystickButton(driverStick, button);
    }

    public JoystickButton setOperatorCommand(int button) {
        return new JoystickButton(operatorStick, button);
    }

    private double deadband(double value) {
        double deadband = 0.1;
        if (Math.abs(value) > deadband)
            if (value > 0.0) return (value - deadband) / (1.0 - deadband);
            else return (value + deadband) / (1.0 - deadband);
        else
            return 0.0;
    }

    private double modifyAxis(double value) {
        value = deadband(value);
        value = Math.copySign(value * value, value); // TODO: Make cubed?

        return value;
    }
}