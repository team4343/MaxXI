package frc.robot.util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.constants.MotorConstants.DriveConstants.KINEMATICS;
import static frc.robot.subsystems.DrivetrainSubsystem.getModulePositions;
import static frc.robot.subsystems.DrivetrainSubsystem.gyroscope;

public class Odometry implements Loggable {
    private SwerveDrivePoseEstimator m_poseEstimator;

    @Log.Field2d(name = "Estimated position", tabName = "Odometry")
    private final Field2d m_field = new Field2d();

    public Odometry() {
        m_poseEstimator = new SwerveDrivePoseEstimator(
                KINEMATICS,
                gyroscope.getRotation2d(),
                getModulePositions(),
                new Pose2d(0.0, 0.0, new Rotation2d(0.0))
        );

        var tab = Shuffleboard.getTab("Odometry");
        tab.addNumber("X", () -> m_poseEstimator.getEstimatedPosition().getX());
        tab.addNumber("Y", () -> m_poseEstimator.getEstimatedPosition().getY());
        tab.addNumber("theta", () -> m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetRotation() {
        m_poseEstimator.resetPosition(gyroscope.getRotation2d(), getModulePositions(), getPose());
    }

    public void updateOdometry() {
        m_field.setRobotPose(getPose());
        m_poseEstimator.update(gyroscope.getRotation2d(), getModulePositions());
    }

}