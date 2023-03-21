package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static frc.robot.constants.VisionConstants.*;

public final class LocationConstants {
    public static final double FIELD_WIDTH = Units.feetToMeters(27.0);
    public static final double FIELD_LENGTH = Units.feetToMeters(54.0);

    // Loading station positions
    public static final RobotPositionConstants RED_LOADING_STATION_GROUND = new RobotPositionConstants(TAG4, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    public static final RobotPositionConstants BLUE_LOADING_STATION = new RobotPositionConstants(TAG5, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

    // PLacing positions
    public static final RobotPositionConstants RED_GRID = new RobotPositionConstants(TAG1, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

    public static void getClosetGrid(Pose2d robotPose) {
        /*
         * Chose the closest grid to the robot to place.
         * TODO - Implement this
         */
    }

    public static final class RobotPositionConstants {
        public final AprilTag tag;
        public final Pose2d pose;

        public RobotPositionConstants(AprilTag tag, Pose2d pose) {
            this.tag = tag;
            this.pose = pose;
        }
    }
}

