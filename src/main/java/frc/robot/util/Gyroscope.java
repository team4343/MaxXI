package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Gyroscope {
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP); // NavX connected over MXP

    public Gyroscope() {
        // Send gyro information, too.
        var tab = Shuffleboard.getTab("Gyroscope");
        tab.addNumber("Yaw", m_navx::getAngle);
        tab.addNumber("Pitch", m_navx::getPitch);
        tab.addNumber("Roll", m_navx::getRoll);
        tab.addNumber("Rotation", () -> getRotation2d().getDegrees());
        tab.addBoolean("Mag calibrated", m_navx::isMagnetometerCalibrated);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is
     * currently facing to the 'forwards' direction.
     */
    public void reset() {
        m_navx.zeroYaw();

    }

    // Equivalent to the "Yaw".
    // As the robot turns left (CCW), the angle should increase.
    public Rotation2d getRotation2d() {
        return (m_navx.getRotation2d());
        // if (m_navx.isMagnetometerCalibrated())
        //     // We will only get valid fused headings if the magnetometer is calibrated
        //     return Rotation2d.fromDegrees(360 - m_navx.getFusedHeading());

        // // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        // return Rotation2d.fromDegrees(m_navx.getYaw() + 180); // TODO: verify this.
    }

    public float getGyroscopePitch() {
        return m_navx.getPitch();
    }

    public float getGyroscopeRoll() {
        return m_navx.getRoll();
    }
}
