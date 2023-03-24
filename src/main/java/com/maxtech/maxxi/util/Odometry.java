package com.maxtech.maxxi.util;

import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Odometry {
    private final PhotonCameraWrapper photonCameraWrapper;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator =
            new SwerveDrivePoseEstimator(null, null, null, null);

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<SwerveModulePosition[]> swerveModulePositionSupplier;

    private final Field2d m_field = new Field2d();

    public Odometry(Supplier<Pose2d> poseSupplier, Supplier<SwerveModulePosition[]> swerveModulePositionSupplier, PhotonCameraWrapper photonCameraWrapper) {
        this.photonCameraWrapper = photonCameraWrapper;
        this.poseSupplier = poseSupplier;
        this.swerveModulePositionSupplier = swerveModulePositionSupplier;

        var tab = Shuffleboard.getTab("Odometry");

        tab.add(m_field);
    }

    public void periodic() {
        // Encoder-based

        swerveDrivePoseEstimator.update(poseSupplier.get().getRotation(), swerveModulePositionSupplier.get());


        // Camera-based

        Optional<EstimatedRobotPose> result = photonCameraWrapper
                .getEstimatedGlobalPose(swerveDrivePoseEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            swerveDrivePoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
                    camPose.timestampSeconds);
            m_field.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
        } else {
            // move it way off the screen to make it disappear
            m_field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }

        // Update actual position on camera.

        m_field.getObject("Actual Pos").setPose(swerveDrivePoseEstimator.getEstimatedPosition());
        m_field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
    }
}
