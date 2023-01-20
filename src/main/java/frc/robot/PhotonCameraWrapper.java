// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
        public PhotonCamera photonCamera;
        public PhotonPoseEstimator robotPoseEstimator;
        public boolean detecting = false;

        public PhotonCameraWrapper() {
                // Set up a test arena of two apriltags at the center of each driver station set
                final AprilTag tag08 = new AprilTag(8, new Pose3d(new Pose2d(FieldConstants.length,
                                FieldConstants.width / 2.0, Rotation2d.fromDegrees(180))));
                final AprilTag tag01 = new AprilTag(1, new Pose3d(new Pose2d(0.0,
                                FieldConstants.width / 2.0, Rotation2d.fromDegrees(0.0))));
                ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
                atList.add(tag08);
                atList.add(tag01);

                // TODO - once 2023 happens, replace this with just loading the 2023 field
                // arrangement
                AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, FieldConstants.length,
                                FieldConstants.width);

                // Forward Camera
                photonCamera = new PhotonCamera(VisionConstants.cameraName); // Change the name of
                                                                             // your camera here to
                                                                             // whatever it is in
                                                                             // the
                // PhotonVision UI.

                // ... Add other cameras here
                // Multiple cameras???

                // Assemble the list of cameras & mount locations
                photonCamera = new PhotonCamera(
                                VisionConstants.cameraName); // Change the name of your camera here to whatever it is in
                                                             // the

                robotPoseEstimator = new PhotonPoseEstimator(atfl,
                                PoseStrategy.LOWEST_AMBIGUITY, photonCamera, VisionConstants.robotToCam);
        }

        /**
         * @param estimatedRobotPose The current best guess at robot pose
         * @return A pair of the fused camera observations to a single Pose2d on the
         *         field, and the
         *         time of the observation. Assumes a planar field and the robot is
         *         always firmly on
         *         the ground
         */
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
                robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
                return robotPoseEstimator.update();
        }
}
