// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PhotonCameraWrapper;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is useful during
         * initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;
        // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk4i standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
                        * SdsModuleConfigurations.MK4I_L1.getDriveReduction()
                        * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;

        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
                        MAX_VELOCITY_METERS_PER_SECOND
                                        / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                                        DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // By default we use a Pigeon for our gyroscope. But if you use another
        // gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        // FIXME Remove if you are using a Pigeon
        // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
        // FIXME Uncomment if you are using a NavX
        private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private final SwerveDrivePoseEstimator m_poseEstimator;

        private final PhotonCameraWrapper pcw;

        private final Field2d m_field = new Field2d(); 

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                // There are 4 methods you can call to create your swerve modules.
                // The method you use depends on what motors you are using.
                //
                // Mk4iSwerveModuleHelper.createFalcon500(...)
                // Your module has two Falcon 500s on it. One for steering and one for driving.
                //
                // Mk4iSwerveModuleHelper.createNeo(...)
                // Your module has two NEOs on it. One for steering and one for driving.
                //
                // Mk4iSwerveModuleHelper.createFalcon500Neo(...)
                // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
                // and the NEO is for steering.
                //
                // Mk4iSwerveModuleHelper.createNeoFalcon500(...)
                // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
                // Falcon 500 is for steering.
                //
                // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
                // class.

                // By default we will use Falcon 500s in standard configuration. But if you use
                // a different configuration or motors
                // you MUST change it. If you do not, your code will crash on startup.
                // FIXME Setup motor configuration
                m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                                // This parameter is optional, but will allow you to see the current
                                // state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4).withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear
                                // configuration
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                // This is the ID of the drive motor
                                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                // This is the ID of the steer motor
                                FRONT_LEFT_MODULE_STEER_MOTOR,
                                // This is the ID of the steer encoder
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                // This is how much the steer encoder is offset from true zero (In
                                // our case,
                                // zero is facing straight forward)
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4).withPosition(2, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1, FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4).withPosition(4, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                BACK_LEFT_MODULE_DRIVE_MOTOR_OG, BACK_LEFT_MODULE_STEER_MOTOR_OG,
                                BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4).withPosition(6, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1, BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);

                // Connect to the Photon camera.
                pcw = new PhotonCameraWrapper();

                m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_navx.getRotation2d(),
                                new SwerveModulePosition[] {
                                 new SwerveModulePosition(m_frontLeftModule.getDriveDistance(),
                                                Rotation2d.fromDegrees(
                                                                m_frontLeftModule.getSteerAngle())),
                                new SwerveModulePosition(m_frontRightModule.getDriveDistance(),
                                                Rotation2d.fromDegrees(
                                                                m_frontRightModule.getSteerAngle())),
                                new SwerveModulePosition(m_backLeftModule.getDriveDistance(),
                                                Rotation2d.fromDegrees(
                                                                m_backLeftModule.getSteerAngle())),
                                new SwerveModulePosition(m_backRightModule.getDriveDistance(),
                                                Rotation2d.fromDegrees(
                                                                m_backRightModule.getSteerAngle()))},
                                new Pose2d());

                // Send gyro information, too.
                var navxTab = tab.getLayout("NavX", BuiltInLayouts.kList).withSize(2, 4)
                                .withPosition(8, 0);

                navxTab.addNumber("Yaw", () -> m_navx.getYaw());
                navxTab.addNumber("Pitch", () -> m_navx.getPitch());
                navxTab.addNumber("Roll", () -> m_navx.getRoll());

                // Send the field data, too.
                tab.add(m_field);
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the robot is
         * currently facing to the 'forwards' direction.
         */
        public void zeroGyroscope() {
                // FIXME Remove if you are using a Pigeon
                // m_pigeon.setFusedHeading(0.0);

                // FIXME Uncomment if you are using a NavX
                m_navx.zeroYaw();
        }

        // Equivalent to the "Yaw".
        public Rotation2d getGyroscopeRotation() {
                // FIXME Remove if you are using a Pigeon
                // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

                // FIXME Uncomment if you are using a NavX
                if (m_navx.isMagnetometerCalibrated()) {
                        // We will only get valid fused headings if the magnetometer is calibrated
                        return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                }

                // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
        }

        public float getGyroscopePitch() {
                return m_navx.getPitch();
        }

        public float getGyroscopeRoll() {
                return m_navx.getRoll();
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(states[0].speedMetersPerSecond
                                / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond
                                / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND
                                * MAX_VOLTAGE, states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond
                                / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

                updateOdometry();

                // Send the current estimated pose to the field object, which will send it to Shuffleboard.
                m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        }

        public void updateOdometry() {
                m_poseEstimator.update(m_navx.getRotation2d(), new SwerveModulePosition[] {
                                new SwerveModulePosition(m_frontLeftModule.getDriveDistance(),
                                                Rotation2d.fromDegrees(
                                                                m_frontLeftModule.getSteerAngle())),
                                new SwerveModulePosition(m_frontRightModule.getDriveDistance(),
                                                Rotation2d.fromDegrees(
                                                                m_frontRightModule.getSteerAngle())),
                                new SwerveModulePosition(m_backLeftModule.getDriveDistance(),
                                                Rotation2d.fromDegrees(
                                                                m_backLeftModule.getSteerAngle())),
                                new SwerveModulePosition(m_backRightModule.getDriveDistance(),
                                                Rotation2d.fromDegrees(
                                                                m_backRightModule.getSteerAngle()))});

                // Also apply vision measurements. We use 0.3 seconds in the past as an example
                // -- on
                // a real robot, this must be calculated based either on latency or timestamps.
                Pair<Pose2d, Double> result =
                                pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
                var camPose = result.getFirst();
                var camPoseObsTime = result.getSecond();
                if (camPose != null) {
                        m_poseEstimator.addVisionMeasurement(camPose, camPoseObsTime);
                }
       }
}
