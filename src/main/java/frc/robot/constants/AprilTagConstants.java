package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

// https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf
public final class AprilTagConstants {
    public static final AprilTag tag01 = new AprilTag(1, new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d(0,0, Math.PI)));
    public static final AprilTag tag02 = new AprilTag(2, new Pose3d(15.513558, 2.748026, 0.462788, new Rotation3d(0,0, Math.PI)));
    public static final AprilTag tag03 = new AprilTag(3, new Pose3d(15.513558, 4.424426, 0.462788, new Rotation3d(0,0, Math.PI)));
    public static final AprilTag tag04 = new AprilTag(4, new Pose3d(16.178784, 6.749796, 0.692912, new Rotation3d(0,0, Math.PI)));
    public static final AprilTag tag05 = new AprilTag(5, new Pose3d(0.3619500, 6.749796, 0.692912, new Rotation3d(0,0, 0)));
    public static final AprilTag tag06 = new AprilTag(6, new Pose3d(1.0274300, 4.424426, 0.462788, new Rotation3d(0,0, 0)));
    public static final AprilTag tag07 = new AprilTag(7, new Pose3d(1.0274300, 2.748026, 0.462788, new Rotation3d(0,0, 0)));
    public static final AprilTag tag08 = new AprilTag(8, new Pose3d(1.0274300, 1.071626, 0.462788, new Rotation3d(0,0, 0)));
}
