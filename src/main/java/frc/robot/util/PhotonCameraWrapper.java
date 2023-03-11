package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.constants.LocationConstants;
import frc.robot.constants.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.ArrayList;
import java.util.Optional;

import static frc.robot.constants.AprilTagConstants.*;
import static frc.robot.constants.AprilTagConstants.tag08;

public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator robotPoseEstimator;

    private double x = 0;
    private double y = 0;
    private double t = 0;

    public PhotonCameraWrapper() {
        // Set up a test arena of two apriltags at the center of each driver station
        ArrayList<AprilTag> atList = new ArrayList<>();
        atList.add(tag01);
        atList.add(tag02);
        atList.add(tag03);
        atList.add(tag04);
        atList.add(tag05);
        atList.add(tag06);
        atList.add(tag07);
        atList.add(tag08);

        AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(atList, LocationConstants.kFieldLength, LocationConstants.kFieldWidth);

        // Forward Camera
        photonCamera = new PhotonCamera(VisionConstants.cameraName);
        robotPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, photonCamera, VisionConstants.robotToCam);

        var tab = Shuffleboard.getTab("PhotonVision");
        tab.addNumber("Camera-estimated X", this::getX);
        tab.addNumber("Camera-estimated Y", this::getY);
        tab.addNumber("Camera-estimated T", this::getT);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getT() {
        return t;
    }

    /**
     * @param prevEstimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the
     * time of the observation. Assumes a planar field and the robot is always firmly on
     * the ground
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