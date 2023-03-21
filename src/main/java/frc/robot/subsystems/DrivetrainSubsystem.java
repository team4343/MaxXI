// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

import static frc.robot.constants.DriveConstants.*;
import static frc.robot.constants.MotorConstants.*;

/**
 * The DrivetrainSubsystem class is the subsystem that controls the robot's drivetrain.
 * <p>
 * This handles the setup and main control of the odometry and vision tracking.
 */
public class DrivetrainSubsystem extends SubsystemBase implements Loggable {
    private static ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private static final ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    private static final SwerveModule frontRightModule = new MkSwerveModuleBuilder()
        .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0)) // TODO Remove these
        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
        .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, FRONT_RIGHT_STEER_MOTOR)
        .withSteerEncoderPort(FRONT_RIGHT_STEER_ENCODER)
        .withSteerOffset(FRONT_RIGHT_STEER_OFFSET).build();

    private static final SwerveModule backLeftModule = new MkSwerveModuleBuilder()
        .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0))
        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
        .withDriveMotor(MotorType.FALCON, BACK_LEFT_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, BACK_LEFT_STEER_MOTOR)
        .withSteerEncoderPort(BACK_LEFT_STEER_ENCODER)
        .withSteerOffset(BACK_LEFT_STEER_OFFSET).build();

    private static final SwerveModule backRightModule = new MkSwerveModuleBuilder()
        .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0))
        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
        .withDriveMotor(MotorType.FALCON, BACK_RIGHT_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, BACK_RIGHT_STEER_MOTOR)
        .withSteerEncoderPort(BACK_RIGHT_STEER_ENCODER)
        .withSteerOffset(BACK_RIGHT_STEER_OFFSET).build();

    private static final SwerveModule frontLeftModule = new MkSwerveModuleBuilder()
        .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0))
        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
        .withDriveMotor(MotorType.FALCON, FRONT_LEFT_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, FRONT_LEFT_STEER_MOTOR)
        .withSteerEncoderPort(FRONT_LEFT_STEER_ENCODER)
        .withSteerOffset(FRONT_LEFT_STEER_OFFSET).build();

    /**
     * Run the drivetrain at the given chassis speeds and update the odometry.
     * This is the main control of the drivetrain.
     */
    @Override
    public void periodic() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // This is to make sure that the robot doesn't go faster than it can
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    /**
     * @return The current module positions.
     */
    public static SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeftModule.getDriveDistance(), Rotation2d.fromRadians(frontLeftModule.getSteerAngle())),
            new SwerveModulePosition(frontRightModule.getDriveDistance() , Rotation2d.fromRadians(frontRightModule.getSteerAngle())),
            new SwerveModulePosition(backLeftModule.getDriveDistance() , Rotation2d.fromRadians(backLeftModule.getSteerAngle())),
            new SwerveModulePosition(backRightModule.getDriveDistance() , Rotation2d.fromRadians(backRightModule.getSteerAngle()))
        };
    }

    /**
     * Set the chassis speeds directly.
     *
     * @param chassisSpeeds The chassis speeds.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        DrivetrainSubsystem.chassisSpeeds = chassisSpeeds;
    }

    /**
     * Set the chassis module states directly.
     *
     * @param states The module states.
     */
    public void driveWithStates(SwerveModuleState[] states) {
        frontLeftModule.set(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }

}

