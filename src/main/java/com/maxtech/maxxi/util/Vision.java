package com.maxtech.maxxi.util;

import com.maxtech.maxxi.constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static com.maxtech.maxxi.constants.LocationConstants.FIELD_LENGTH;
import static com.maxtech.maxxi.constants.LocationConstants.FIELD_WIDTH;
import static com.maxtech.maxxi.constants.VisionConstants.frameTimeout;
import static com.maxtech.maxxi.constants.VisionConstants.getAprilTagList;

public class Vision {
    private final AprilTagFieldLayout layout;
    private final PhotonCamera camera;
    private PhotonTrackedTarget latestTarget;
    private double latestTargetTime;

    public Vision() {
        this.layout = new AprilTagFieldLayout(getAprilTagList(), FIELD_LENGTH, FIELD_WIDTH);
        this.camera = new PhotonCamera(VisionConstants.cameraName);
    }

    /**
     * Get the robot's pose relative to the target.
     *
     * @return The robot's pose relative to the target.
     */
    public VisionPoseResult getPose() {
        // Get the latest result from the camera.
        PhotonPipelineResult result = camera.getLatestResult();
        this.latestTargetTime = result.getTimestampSeconds();
        this.latestTarget = result.getBestTarget();

        // If there are no accurate targets, return null.
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget == null) return null;
        if (bestTarget.getPoseAmbiguity() > 0.5) return null;

        Transform3d camToTargetTrans = bestTarget.getBestCameraToTarget();
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(latestTarget.getFiducialId());

        if (tagPoseOptional.isEmpty()) return null;
        else {
            Pose3d tagPose = tagPoseOptional.get();
            Pose3d camPose = tagPose.transformBy(camToTargetTrans.inverse());
            Pose2d robotPose = camPose.transformBy(VisionConstants.robotToCam).toPose2d();

            return new VisionPoseResult(robotPose, this.latestTargetTime);
        }
    }

    public PhotonTrackedTarget getLatestTarget() {
        return latestTarget;
    }


    public boolean hasTarget() {
        if (latestTargetTime > camera.getLatestResult().getTimestampSeconds() - frameTimeout)
            latestTarget = null;
        return latestTarget != null;
    }


}
