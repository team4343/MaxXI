package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.List;

public final class VisionConstants {
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(.1524, .0889, .762), new Rotation3d(0, 0, 0));
    public static final String cameraName = "Microsoft_LifeCam_HD-3000";
    public static final double frameTimeout = 0.2; // Seconds

    // https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf
    // RED
    public static final AprilTag TAG1 = new AprilTag(1, new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d(0,0, Math.PI)));
    public static final AprilTag TAG2 = new AprilTag(2, new Pose3d(15.513558, 2.748026, 0.462788, new Rotation3d(0,0, Math.PI)));
    public static final AprilTag TAG3 = new AprilTag(3, new Pose3d(15.513558, 4.424426, 0.462788, new Rotation3d(0,0, Math.PI)));
    public static final AprilTag TAG4 = new AprilTag(4, new Pose3d(16.178784, 6.749796, 0.692912, new Rotation3d(0,0, Math.PI)));
    // BLUE
    public static final AprilTag TAG5 = new AprilTag(5, new Pose3d(0.3619500, 6.749796, 0.692912, new Rotation3d(0,0, 0)));
    public static final AprilTag TAG6 = new AprilTag(6, new Pose3d(1.0274300, 4.424426, 0.462788, new Rotation3d(0,0, 0)));
    public static final AprilTag TAG7 = new AprilTag(7, new Pose3d(1.0274300, 2.748026, 0.462788, new Rotation3d(0,0, 0)));
    public static final AprilTag TAG8 = new AprilTag(8, new Pose3d(1.0274300, 1.071626, 0.462788, new Rotation3d(0,0, 0)));

    public static List<AprilTag> getAprilTagList() {
        return List.of(TAG1, TAG2, TAG3, TAG4, TAG5, TAG6, TAG7, TAG8);
    }
}