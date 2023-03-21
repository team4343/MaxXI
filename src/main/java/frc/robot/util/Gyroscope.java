package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Gyroscope {
    private final AHRS navx;

    public Gyroscope() {
        this.navx = new AHRS(SPI.Port.kMXP);
        navx.setAngleAdjustment(0);

        var tab = Shuffleboard.getTab("Gyroscope");
        tab.addNumber("Pitch", navx::getPitch);
        tab.addNumber("Roll", navx::getRoll);
        tab.addNumber("Rotation", () -> getRotation2d().getDegrees());
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is
     * currently facing to the 'forwards' direction.
     */
    public void reset() {
        navx.zeroYaw();
    }

    /** The rotation angle is the angle between the X axis and the plane formed by the X and Z axes. */
    public Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }

    /** The pitch angle is the angle between the X axis and the plane formed by the X and Y axes. */
    public float getGyroscopePitch() {
        return navx.getPitch();
    }

    /** The yaw angle is the angle between the Y axis and the plane formed by the Y and X axes. */
    public float getGyroscopeRoll() {
        return navx.getRoll();
    }
}
