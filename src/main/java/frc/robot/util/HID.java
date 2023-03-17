package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.MotorConstants;

public class HID {
    private final Joystick m_driverStick;
//    private final Joystick m_OperatorStick;
    public static int alliance_modifier = 1;
    private final double deadband = 0.1;

    public HID(int driverPort, int OperatorPort) {
        m_driverStick = new Joystick(driverPort);
//        m_OperatorStick = new Joystick(OperatorPort);
    }

    public double getDriverX() {
         return modifyAxis(m_driverStick.getY()) * MotorConstants.DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * alliance_modifier;
    }
    public double getDriverY() {

         return modifyAxis(m_driverStick.getX()) * MotorConstants.DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * alliance_modifier;
    }

    public double getDriverT() {
        return modifyAxis(m_driverStick.getTwist()) * MotorConstants.DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * alliance_modifier;
    }

    public JoystickButton setDriverCommand(int button) {
        return new JoystickButton(m_driverStick, button);
    }
//    public JoystickButton setOperatorCommand(int button) {
//        return new JoystickButton(m_OperatorStick, button);
//    }
    public void toggle() {
        this.alliance_modifier *= -1;
    }

    private double deadband(double value) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private double modifyAxis(double value) {
        // Deadband
        value = deadband(value);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}