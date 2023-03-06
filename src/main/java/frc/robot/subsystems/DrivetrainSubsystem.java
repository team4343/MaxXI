// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer.WorkBenchModerator;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class DrivetrainSubsystem extends SubsystemBase implements Loggable {
    private final WorkBenchModerator m_workBenchModerator;

    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public final PhotonCameraWrapper pcw;
    public final Gyroscope gyroscope;
    public final Odometry odometry;

    public DrivetrainSubsystem(WorkBenchModerator m_workBenchModerator) {
        this.m_workBenchModerator = m_workBenchModerator;

        if (m_workBenchModerator.isReal()) {
            m_frontLeftModule = new MkSwerveModuleBuilder()
                    .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                            .withSize(2, 4).withPosition(0, 0))
                    .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                    .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR)
                    .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
                    .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
                    .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET).build();

            m_frontRightModule = new MkSwerveModuleBuilder()
                    .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                            .withSize(2, 4).withPosition(2, 0))
                    .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                    .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                    .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
                    .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
                    .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET).build();

            m_backLeftModule = new MkSwerveModuleBuilder()
                    .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                            .withSize(2, 4).withPosition(4, 0))
                    .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                    .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR_OG)
                    .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR_OG)
                    .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
                    .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET).build();

            m_backRightModule = new MkSwerveModuleBuilder()
                    .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                            .withSize(2, 4).withPosition(6, 0))
                    .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                    .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR)
                    .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
                    .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
                    .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET).build();

            pcw = new PhotonCameraWrapper();
            gyroscope = new Gyroscope();
            odometry = new Odometry();
        } else {
            m_frontLeftModule = null;
            m_frontRightModule = null;
            m_backLeftModule = null;
            m_backRightModule = null;

            pcw = null;
            gyroscope = null;
            odometry = null;
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public void driveWithStates(SwerveModuleState[] states) {
        m_chassisSpeeds = KINEMATICS.toChassisSpeeds(states);
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        if (m_workBenchModerator.isReal()) {
            m_frontLeftModule.set(
                    states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                    states[0].angle.getRadians());
            m_frontRightModule.set(
                    states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                    states[1].angle.getRadians());
            m_backLeftModule.set(
                    states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                    states[2].angle.getRadians());
            m_backRightModule.set(
                    states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                    states[3].angle.getRadians());

            odometry.updateOdometry();
        }
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(new InstantCommand(() -> {
            odometry.setFieldTrajectory(traj);
            // Reset odometry for the first path you run during auto
            if (isFirstPath) {

                // this.resetOdometry(traj.getInitialHolonomicPose());
            }
        }), new PPSwerveControllerCommand(traj, odometry::getPose, // Pose supplier
                KINEMATICS, // SwerveDriveKinematics
                new PIDController(.2, 0, 0), // X controller. Tune these values for
                                             // your
                                             // robot. Leaving them 0 will only use
                                             // feedforwards.
                new PIDController(0.2, 0, 0), // Y controller (usually the same values
                                              // as
                                              // X controller)
                new PIDController(0.2, 0, 0), // Rotation controller. Tune these
                                              // values
                                              // for your robot. Leaving them 0 will
                                              // only
                                              // use feedforwards.
                this::driveWithStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on
                      // alliance
                      // color. Optional, defaults to true
                this // Requires this drive subsystem
        ));
    }

    public Command constructLiveTrajectoryCommand(ArmSubsystem armSubsystem) {
        var commands = new SequentialCommandGroup();

        var handle = NetworkTableInstance.getDefault();
        var stepAmountHandle = handle.getEntry("/AutoSelector/steps/amount");
        var steps = stepAmountHandle.getInteger(0);

        var prevPose = odometry.getPose();

        for (var i = 1; i <= steps; i++) {
            var x = handle.getEntry("/AutoSelector/steps/" + i + "/x").getDouble(0);
            var y = handle.getEntry("/AutoSelector/steps/" + i + "/y").getDouble(0);
            var t = handle.getEntry("/AutoSelector/steps/" + i + "/t").getDouble(0);
            var maxAccel = handle.getEntry("/AutoSelector/steps/" + i + "/maxVel").getDouble(0.5);
            var maxVel = handle.getEntry("/AutoSelector/steps/" + i + "/maxAccel").getDouble(0.5);
            var stateName =
                    handle.getEntry("/AutoSelector/steps/" + i + "/state").getString("Rest");

            var state = armSubsystem.deserializeStateFromString(stateName);
            System.out.println("State is " + state);

            var point = new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(t));

            ArrayList<PathPoint> points = new ArrayList<>();
            points.add(new PathPoint(prevPose.getTranslation(), prevPose.getRotation()));
            points.add(point);

            prevPose = new Pose2d(new Translation2d(x, y), new Rotation2d(t));

            PathPlannerTrajectory trajectory =
                    PathPlanner.generatePath(new PathConstraints(maxVel, maxAccel), points);

            commands.addCommands(new ParallelCommandGroup(new InstantCommand(() -> {
                armSubsystem.setState(state);
                System.out.println("Set state to " + state.toString());
            }), new PPSwerveControllerCommand(trajectory, odometry::getPose, // Pose
                    // supplier
                    KINEMATICS, // SwerveDriveKinematics
                    new PIDController(.6, 0, 0), // X controller. Tune these values for
                                                 // your
                                                 // robot. Leaving them 0 will only use
                                                 // feedforwards.
                    new PIDController(0.6, 0, 0), // Y controller (usually the same values
                                                  // as
                                                  // X controller)
                    new PIDController(0.6, 0, 0), // Rotation controller. Tune these
                                                  // values
                                                  // for your robot. Leaving them 0 will
                                                  // only
                                                  // use feedforwards.
                    this::driveWithStates, // Module states consumer
                    true, // Should the path be automatically mirrored depending on
                          // alliance
                          // color. Optional, defaults to true
                    this // Requires this drive subsystem
            )));
        }

        return commands;
    }

    // The conversion ratio between drive rotations and meters. Goes from motor rots
    // -> wheel rots -> meters.
    private static double rotationsToMetersRatio = 1;

    public class Gyroscope {
        private final AHRS m_navx = new AHRS(SPI.Port.kMXP); // NavX connected over MXP

        public Gyroscope() {
            // Send gyro information, too.
            var tab = Shuffleboard.getTab("Gyroscope");

            tab.addNumber("Yaw", () -> m_navx.getYaw());
            tab.addNumber("Pitch", () -> m_navx.getPitch());
            tab.addNumber("Roll", () -> m_navx.getRoll());
            tab.addNumber("Rotation", () -> getRotation2d().getDegrees());
            tab.addBoolean("Mag calibrated", () -> m_navx.isMagnetometerCalibrated());
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
            if (m_navx.isMagnetometerCalibrated()) {
                // We will only get valid fused headings if the magnetometer is
                // calibrated
                return Rotation2d.fromDegrees(360 - m_navx.getFusedHeading());
            }

            // We have to invert the angle of the NavX so that rotating the robot
            // counter-clockwise makes the angle increase.
            // TODO: verify this.
            return Rotation2d.fromDegrees(m_navx.getYaw() + 180);
        }

        public float getGyroscopePitch() {
            return m_navx.getPitch();
        }

        public float getGyroscopeRoll() {
            return m_navx.getRoll();
        }
    }

    public class Odometry implements Loggable {
        private final SwerveDrivePoseEstimator m_poseEstimator;

        @Log.Field2d(name = "Estimated position", tabName = "Odometry")
        private final Field2d m_field = new Field2d();

        public Odometry() {
            m_poseEstimator =
                    new SwerveDrivePoseEstimator(KINEMATICS, gyroscope.getRotation2d(),
                            new SwerveModulePosition[] {new SwerveModulePosition(
                                    m_frontLeftModule.getDriveDistance() * rotationsToMetersRatio,
                                    Rotation2d.fromRadians(m_frontLeftModule.getSteerAngle())),
                                    new SwerveModulePosition(
                                            m_frontRightModule.getDriveDistance()
                                                    * rotationsToMetersRatio,
                                            Rotation2d.fromRadians(
                                                    m_frontRightModule.getSteerAngle())),
                                    new SwerveModulePosition(
                                            m_backLeftModule.getDriveDistance()
                                                    * rotationsToMetersRatio,
                                            Rotation2d
                                                    .fromRadians(m_backLeftModule.getSteerAngle())),
                                    new SwerveModulePosition(
                                            m_backRightModule.getDriveDistance()
                                                    * rotationsToMetersRatio,
                                            Rotation2d.fromRadians(
                                                    m_backRightModule.getSteerAngle()))},
                            new Pose2d(0., 0., new Rotation2d(0.)));

            var tab = Shuffleboard.getTab("Odometry");

            tab.addNumber("X", () -> m_poseEstimator.getEstimatedPosition().getX());
            tab.addNumber("Y", () -> m_poseEstimator.getEstimatedPosition().getY());
            tab.addNumber("theta",
                    () -> m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

            tab.addNumber("FL distance", m_frontLeftModule::getDriveDistance);
        }

        public void setFieldTrajectory(Trajectory trajectory) {
            m_field.getObject("traj").setTrajectory(trajectory);
        }

        public Pose2d getPose() {
            return m_poseEstimator.getEstimatedPosition();
        }

        public void updateOdometry() {
            m_field.setRobotPose(getPose());

            m_poseEstimator
                    .update(gyroscope.getRotation2d(),
                            new SwerveModulePosition[] {new SwerveModulePosition(
                                    m_frontLeftModule.getDriveDistance() * rotationsToMetersRatio,
                                    Rotation2d.fromRadians(m_frontLeftModule.getSteerAngle())),
                                    new SwerveModulePosition(
                                            m_frontRightModule.getDriveDistance()
                                                    * rotationsToMetersRatio,
                                            Rotation2d.fromRadians(
                                                    m_frontRightModule.getSteerAngle())),
                                    new SwerveModulePosition(
                                            m_backLeftModule.getDriveDistance()
                                                    * rotationsToMetersRatio,
                                            Rotation2d
                                                    .fromRadians(m_backLeftModule.getSteerAngle())),
                                    new SwerveModulePosition(
                                            m_backRightModule.getDriveDistance()
                                                    * rotationsToMetersRatio,
                                            Rotation2d.fromRadians(
                                                    m_backRightModule.getSteerAngle()))});

            // Also apply vision measurements. We use 0.3 seconds in the past as an
            // example
            // -- on
            // a real robot, this must be calculated based either on latency or
            // timestamps.
            Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(getPose());
            if (result.isPresent()) {
                EstimatedRobotPose camPose = result.get();

                // Workaround for SwerveDrivePoseEstimator
                // ConccurentModificationException
                var currentTime = Timer.getFPGATimestamp();
                var camTime = camPose.timestampSeconds;

                if (currentTime - camTime < 1.5) {
                    m_poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
                            camPose.timestampSeconds);
                } else {
                    System.out.println("Received outdated vision measurement, current time "
                            + currentTime + ", pose info time " + camTime);
                }
            }
        }
    }

    public class PhotonCameraWrapper {
        public PhotonCamera photonCamera;
        public PhotonPoseEstimator robotPoseEstimator;

        private double x = 0;
        private double y = 0;
        private double t = 0;

        public PhotonCameraWrapper() {
            // Set up a test arena of two apriltags at the center of each driver station
            // set
            final AprilTag tag01 =
                    new AprilTag(1, new Pose3d(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))));
            final AprilTag tag08 = new AprilTag(8, new Pose3d(new Pose2d(FieldConstants.length,
                    FieldConstants.width / 2.0, Rotation2d.fromDegrees(180))));
            ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
            atList.add(tag01);
            atList.add(tag08);

            // TODO - once 2023 happens, replace this with just loading the 2023 field
            // arrangement
            AprilTagFieldLayout atfl =
                    new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);

            // Forward Camera
            photonCamera = new PhotonCamera(VisionConstants.cameraName); // Change the
                                                                         // name of
                                                                         // your camera
                                                                         // here to
                                                                         // whatever it
                                                                         // is in
                                                                         // the
            // PhotonVision UI.

            // ... Add other cameras here
            // Multiple cameras???

            // Assemble the list of cameras & mount locations
            photonCamera = new PhotonCamera(VisionConstants.cameraName); // Change the
                                                                         // name of your
                                                                         // camera here
                                                                         // to whatever
                                                                         // it is in
                                                                         // the

            robotPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY,
                    photonCamera, VisionConstants.robotToCam);

            var tab = Shuffleboard.getTab("PhotonVision");
            tab.addNumber("Camera-estimated X", () -> {
                return this.x;
            });
            tab.addNumber("Camera-estimated Y", () -> {
                return this.y;
            });
            tab.addNumber("Camera-estimated T", () -> {
                return this.t;
            });
        }

        /**
         * @param estimatedRobotPose The current best guess at robot pose
         * @return A pair of the fused camera observations to a single Pose2d on the field, and the
         *         time of the observation. Assumes a planar field and the robot is always firmly on
         *         the ground
         */
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            var estimatedPose = robotPoseEstimator.update();

            estimatedPose.ifPresent((EstimatedRobotPose p) -> {
                this.x = p.estimatedPose.getX();
                this.y = p.estimatedPose.getY();
                this.t = 180 / Math.PI * p.estimatedPose.getRotation().getAngle();
            });

            return estimatedPose;
        }
    }
}
