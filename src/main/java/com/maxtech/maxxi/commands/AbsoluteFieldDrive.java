// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import com.maxtech.swervelib.SwerveController;
import com.maxtech.swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.List;
import java.util.function.DoubleSupplier;

import static com.maxtech.maxxi.constants.DriveConstants.*;


public class AbsoluteFieldDrive extends CommandBase
{

  private final DrivetrainSubsystem swerve;
  private final DoubleSupplier  vX, vY, heading;
  private final boolean isOpenLoop;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve  The swerve drivebase subsystem.
   * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
   *                station glass.
   * @param heading DoubleSupplier that supplies the robot's heading angle.
   */
  public AbsoluteFieldDrive(DrivetrainSubsystem swerve,
                            DoubleSupplier vX,
                            DoubleSupplier vY,
                            DoubleSupplier heading,
                            boolean isOpenLoop) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    this.isOpenLoop = isOpenLoop;

    addRequirements(swerve);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the desired chassis speeds based on a 2 joystick module.
    Rotation2d lastHeading = swerve.getHeading();
    Rotation2d headingChange = Rotation2d.fromRadians(heading.getAsDouble() * Math.PI / 2);
    Rotation2d newHeading = lastHeading.plus(headingChange);

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
        vX.getAsDouble(),
        vY.getAsDouble(),
        newHeading
    );

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(
        translation,
        swerve.getFieldVelocity(),
        swerve.getPose(),
        LOOP_TIME,
        ROBOT_MASS,
        List.of(CHASSIS),
        swerve.getSwerveDriveConfiguration()
    );

    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(
        translation,
        desiredSpeeds.omegaRadiansPerSecond,
        true,
        isOpenLoop
    );
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
