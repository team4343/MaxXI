package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static frc.robot.constants.LocationConstants.FIELD_LENGTH;
import static frc.robot.constants.LocationConstants.FIELD_WIDTH;
import static frc.robot.constants.VisionConstants.frameTimeout;
import static frc.robot.constants.VisionConstants.getAprilTagList;

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
     * <p><b>THIS METHOD IS ONLY CALLED FROM ODOMETRY SUBSYSTEM.</b></p>
     *
     * @return The robot's pose relative to the target.
     */
    public VisionPoseResult getPose() {
        // Get the latest result from the camera.
        PhotonPipelineResult result = camera.getLatestResult();
        this.latestTargetTime = result.getTimestampSeconds();
        this.latestTarget = result.getBestTarget();

        // If there are no targets, return null.
        Transform3d camToTargetTrans = result.getBestTarget().getBestCameraToTarget();
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
