package com.maxtech.swervelib.simulation;

import com.maxtech.swervelib.SwerveDrive;
import com.maxtech.swervelib.SwerveModule;
import com.maxtech.swervelib.math.SwerveKinematics2;
import com.maxtech.swervelib.math.SwerveModuleState2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Optional;

/**
 * Simulation for {@link SwerveDrive} IMU.
 */
public class SwerveIMUSimulation
{

  /**
   * Main timer to control movement estimations.
   */
  private final Timer  timer;
  /**
   * The last time the timer was read, used to determine position changes.
   */
  private       double lastTime;
  /**
   * Heading of the robot.
   */
  private       double angle;

  /**
   * Create the swerve drive IMU simulation.
   */
  public SwerveIMUSimulation()
  {
    timer = new Timer();
    timer.start();
    lastTime = timer.get();
  }

  /**
   * Get the estimated angle of the robot.
   *
   * @return {@link Rotation2d} estimation of the robot.
   */
  public Rotation2d getYaw()
  {
    return new Rotation2d(angle);
  }

  /**
   * Pitch is not simulated currently, always returns 0.
   *
   * @return Pitch of the robot as {@link Rotation2d}.
   */
  public Rotation2d getPitch()
  {
    return new Rotation2d();
  }

  /**
   * Roll is not simulated currently, always returns 0.
   *
   * @return Roll of the robot as {@link Rotation2d}.
   */
  public Rotation2d getRoll()
  {
    return new Rotation2d();
  }

  /**
   * Gets the estimated gyro {@link Rotation3d} of the robot.
   *
   * @return The heading as a {@link Rotation3d} angle
   */
  public Rotation3d getGyroRotation3d()
  {
    return new Rotation3d(0, 0, angle);
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in m/s/s. If acceleration isn't supported returns empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  public Optional<Translation3d> getAccel()
  {
    return Optional.empty();
  }

  /**
   * Update the odometry of the simulated {@link SwerveDrive} and post the {@link SwerveModule}
   * states to the {@link Field2d}.
   *
   * @param kinematics  {@link SwerveKinematics2} of the swerve drive.
   * @param states      {@link SwerveModuleState2} array of the module states.
   * @param modulePoses {@link Pose2d} representing the swerve modules.
   * @param field       {@link Field2d} to update.
   */
  public void updateOdometry(
      SwerveKinematics2 kinematics,
      SwerveModuleState2[] states,
      Pose2d[] modulePoses,
      Field2d field)
  {
    angle += kinematics.toChassisSpeeds(states).omegaRadiansPerSecond * (timer.get() - lastTime);
    lastTime = timer.get();
    field.getObject("XModules").setPoses(modulePoses);
  }

  /**
   * Set the heading of the robot.
   *
   * @param angle Angle of the robot in radians.
   */
  public void setAngle(double angle)
  {
    this.angle = angle;
  }
}
