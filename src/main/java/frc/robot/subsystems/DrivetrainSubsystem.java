// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.util.Gyroscope;
import frc.robot.util.Odometry;
import frc.robot.util.PhotonCameraWrapper;
import io.github.oblarg.oblog.Loggable;

import java.util.ArrayList;
import static frc.robot.constants.MotorConstants.DriveConstants.*;

public class DrivetrainSubsystem extends SubsystemBase implements Loggable {
    public static final PhotonCameraWrapper pcw = new PhotonCameraWrapper();
    public static final Gyroscope gyroscope = new Gyroscope();
    public final Odometry odometry = new Odometry();

    private static final ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    private static final SwerveModule m_frontRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET).build();

    private static final SwerveModule m_backLeftModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR_OG)
            .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR_OG)
            .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET).build();

    private static final SwerveModule m_backRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET).build();

    private static final SwerveModule m_frontLeftModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET).build();

    // This is our kinematics object. We initialize it in the constructor.
    private static final PIDController xPIDController = new PIDController(0.6, 0.01, 0.0);
    private static final PIDController yPIDController = new PIDController(0.6, 0.01, 0.0);
    private static final PIDController rPIDController = new PIDController(0.05, 0.005, 0.0);
    private static final double pi_over_two = Math.PI/2.0;  // The conversion ratio between drive rotations and meters. Goes from motor rots -> wheel rots -> meters.
    private static ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public void driveWithStates(SwerveModuleState[] states) {
        m_chassisSpeeds = KINEMATICS.toChassisSpeeds(states);
        m_chassisSpeeds.omegaRadiansPerSecond *= -1;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(
                states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians() - pi_over_two + Math.PI);
        m_frontRightModule.set(
                states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians() - pi_over_two + Math.PI);
        m_backLeftModule.set(
                states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians() - pi_over_two + Math.PI);
        m_backRightModule.set(
                states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians() - pi_over_two + Math.PI);

        odometry.updateOdometry();
    }

    public static SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(m_frontLeftModule.getDriveDistance(), Rotation2d.fromRadians(m_frontLeftModule.getSteerAngle()+ pi_over_two)),
            new SwerveModulePosition(m_frontRightModule.getDriveDistance() , Rotation2d.fromRadians(m_frontRightModule.getSteerAngle()+ pi_over_two)),
            new SwerveModulePosition(m_backLeftModule.getDriveDistance() , Rotation2d.fromRadians(m_backLeftModule.getSteerAngle() + pi_over_two)),
            new SwerveModulePosition(m_backRightModule.getDriveDistance() , Rotation2d.fromRadians(m_backRightModule.getSteerAngle()+ pi_over_two ))
        };
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
        return new PPSwerveControllerCommand(
                        traj,
                        odometry::getPose, // Pose supplier
                        KINEMATICS,
                        xPIDController,
                        yPIDController,
                        rPIDController,
                        this::driveWithStates, // Module states consumer
                        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                        this // Requires this drive subsystem
        );
    }

    public Command move() {
        Pose2d pose = this.odometry.getPose();
        Pose2d next = new Pose2d(pose.getX(), pose.getY() + 0.5, pose.getRotation().plus(new Rotation2d(Math.PI)));

        ArrayList<PathPoint> points = new ArrayList<>();
        points.add(new PathPoint(pose.getTranslation(), pose.getRotation()));
        points.add(new PathPoint(pose.getTranslation(), next.getRotation()));

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(1, .1), points);
        return this.followTrajectoryCommand(trajectory);
    }

    // This generates a sequential command group.


}

