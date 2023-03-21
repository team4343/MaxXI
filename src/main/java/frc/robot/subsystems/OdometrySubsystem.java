package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Gyroscope;
import frc.robot.util.Vision;
import frc.robot.util.VisionPoseResult;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.constants.DriveConstants.*;
import static frc.robot.subsystems.DrivetrainSubsystem.getModulePositions;

public class OdometrySubsystem extends SubsystemBase implements Loggable {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Gyroscope gyroscope;
    private final Vision vision;

    @Log.Field2d(name = "Estimated position", tabName = "Odometry")
    private final Field2d dashboardField = new Field2d();

    public OdometrySubsystem() {
        this.gyroscope = new Gyroscope();
        this.vision = new Vision();

//      TODO - Figure out how to set the origin position. or if its needed.

//        AprilTagFieldLayout.OriginPosition = DriverStation.getAlliance() == DriverStation.Alliance.Blue
//            ? AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide : AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
//        this.layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

        this.poseEstimator = new SwerveDrivePoseEstimator(
            KINEMATICS,
            gyroscope.getRotation2d(),
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs
        );

        var tab = Shuffleboard.getTab("Odometry");
        tab.addNumber("X", () -> poseEstimator.getEstimatedPosition().getX());
        tab.addNumber("Y", () -> poseEstimator.getEstimatedPosition().getY());
        tab.addNumber("R", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    @Override
    public void periodic() {
        VisionPoseResult result = vision.getPose();
        poseEstimator.addVisionMeasurement(result.pose, result.timestamp);
        poseEstimator.update(gyroscope.getRotation2d(), getModulePositions());

        dashboardField.setRobotPose(getPose());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetRotationToGyro() {
        poseEstimator.resetPosition(gyroscope.getRotation2d(), getModulePositions(), getPose());
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(gyroscope.getRotation2d(), getModulePositions(), pose);
    }

    public void resetToVision() {
        if (!vision.hasTarget()) {
            return;
        }
        VisionPoseResult result = vision.getPose();
        resetOdometry(result.pose);
    }

}