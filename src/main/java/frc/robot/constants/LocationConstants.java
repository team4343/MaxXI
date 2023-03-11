package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LocationConstants {
    protected static final double kFieldWidth = 27.0;
    protected static final double kFieldLength = 54.0;
    protected static final double kFieldHeight = 0.0;

    public static final RobotPositionConstant kRedLoadingStation = new RobotPositionConstant(AprilTags.tag04,  new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    public static final RobotPositionConstant kBlueLoadingStation = new RobotPositionConstant(AprilTags.tag05,  new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
}
