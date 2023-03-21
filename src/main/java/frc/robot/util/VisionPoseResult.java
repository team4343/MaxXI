package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionPoseResult {
    public final Pose2d pose;
    public final double timestamp;

    public VisionPoseResult(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
