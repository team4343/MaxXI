package frc.robot.util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

import static frc.robot.constants.MotorConstants.DriveConstants.KINEMATICS;
import static frc.robot.subsystems.DrivetrainSubsystem.*;

public class Odometry implements Loggable {
    private final SwerveDrivePoseEstimator m_poseEstimator;

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

    // Shuffleboard
    public void setFieldTrajectory(Trajectory trajectory) {
        m_field.getObject("traj").setTrajectory(trajectory);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void updateOdometry() {
        m_field.setRobotPose(getPose());

        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // on a real robot, this must be calculated based either on latency or timestamps.
        Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(getPose());
        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            double currentTime = Timer.getFPGATimestamp();
            double camTime = camPose.timestampSeconds;

            if (currentTime - camTime < 0.2)
                m_poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            else
                System.out.println("Received outdated vision measurement, current time " + currentTime + ", pose info time " + camTime);
        }

        m_poseEstimator.update(gyroscope.getRotation2d(), getModulePositions());

    }
}