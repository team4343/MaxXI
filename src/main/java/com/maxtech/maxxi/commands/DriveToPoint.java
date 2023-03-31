package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import com.maxtech.swervelib.SwerveController;
import com.maxtech.swervelib.math.SwerveMath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.List;

import static com.maxtech.maxxi.constants.DriveConstants.*;

public class DriveToPoint extends CommandBase {
    Pose2d desiredPose;
    DrivetrainSubsystem swerve;
    PIDController xController;
    PIDController yController;
    PIDController rController;

    public DriveToPoint(DrivetrainSubsystem drivebase, Pose2d desiredPose) {
        addRequirements(drivebase);
        this.desiredPose = desiredPose;
        this.swerve = drivebase;
    }

    @Override
    public void initialize() {
        this.xController = xAutoPIDConf.createPIDController();
        this.yController = yAutoPIDConf.createPIDController();
        this.rController = rAutoPIDConf.createPIDController();
    }

    @Override
    public void execute() {
        double vX = this.xController.calculate(swerve.getPose().getX()) * -1;
        double vY = this.yController.calculate(swerve.getPose().getY()) * -1;
        double heading = this.rController.calculate(swerve.getPose().getY());

        Rotation2d lastHeading = swerve.getHeading();
        Rotation2d headingChange = Rotation2d.fromRadians(heading * Math.PI / 2);
        Rotation2d newHeading = lastHeading.plus(headingChange);

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
            vX,
            vY,
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
            false
        );


    }


    @Override
    public boolean isFinished() {
        return this.xController.getPositionError() < 1 && this.yController.getPositionError() < 1 && this.xController.getPositionError() < 1;
    }
}
