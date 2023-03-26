// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.maxtech.maxxi.commands.drivebase;

import com.maxtech.swervelib.SwerveController;
import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDrive extends CommandBase
{

  private final DrivetrainSubsystem swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final DoubleSupplier   omega;
  private final BooleanSupplier  driveMode;
  private final boolean          isOpenLoop;
  private final SwerveController controller;
  private final Timer            timer    = new Timer();
  private final boolean          headingCorrection;
  private       double           angle    = 0;
  private       double           lastTime = 0;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(DrivetrainSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
                     BooleanSupplier driveMode, boolean isOpenLoop, boolean headingCorrection)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.isOpenLoop = isOpenLoop;
    this.controller = swerve.getSwerveController();
    this.headingCorrection = headingCorrection;
    if (headingCorrection)
    {
      timer.start();
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (headingCorrection) {
      lastTime = timer.get();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    double xVelocity   = vX.getAsDouble();
    double yVelocity   = vY.getAsDouble();
    double angVelocity = omega.getAsDouble();

    SmartDashboard.putNumber("vX", xVelocity * controller.config.maxSpeed);
    SmartDashboard.putNumber("vY", yVelocity * controller.config.maxSpeed);
    SmartDashboard.putNumber("omega", angVelocity  * controller.config.maxAngularVelocity);


    swerve.drive(
        new Translation2d(
            xVelocity * controller.config.maxSpeed,
            yVelocity * controller.config.maxSpeed),
        angVelocity * controller.config.maxAngularVelocity,
        driveMode.getAsBoolean(),
        isOpenLoop);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
