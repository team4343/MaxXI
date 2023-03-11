package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class LocationConstants {
    public static final double kFieldWidth = Units.feetToMeters(27.0);
    public static final double kFieldLength = Units.feetToMeters(54.0);

    public static final RobotPositionConstants kRedLoadingStation = new RobotPositionConstants(AprilTagConstants.tag04,  new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    public static final RobotPositionConstants kBlueLoadingStation = new RobotPositionConstants(AprilTagConstants.tag05,  new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
}
