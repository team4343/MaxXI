package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.maxtech.maxxi.constants.DriveConstants.rAutoPIDConf;

public class PointRotate extends CommandBase {
    DrivetrainSubsystem drivetrainSubsystem;
    Rotation2d target;
    PIDController controller;

    public PointRotate(DrivetrainSubsystem drivetrainSubsystem, Rotation2d target) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.target = target;
        this.controller = rAutoPIDConf.createPIDController();
        this.controller.setTolerance(Rotation2d.fromDegrees(10).getRadians());
        this.controller.enableContinuousInput(-Math.PI/2, Math.PI/2);
//        this.controller.enableContinuousInput(0, Math.PI);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        Rotation2d heading = this.drivetrainSubsystem.getHeading();
        double rotationOutput =  controller.calculate(heading.getRadians());
        this.drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0,0,  rotationOutput));
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrainSubsystem.swerveDrive.lockPose();
        this.m_requirements.clear();
    }


    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

}