package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;

public class RobotPositionConstant {
    public final AprilTag tag;
    public final Pose2d pose;

    public RobotPositionConstant(AprilTag tag, Pose2d pose) {
        this.tag = tag;
        this.pose = pose;
    }
}