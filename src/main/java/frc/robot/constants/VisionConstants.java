package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants {
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(.1524, .0889, .762), new Rotation3d(0, 0, 0));
    public static final String cameraName = "Microsoft_LifeCam_HD-3000";
}