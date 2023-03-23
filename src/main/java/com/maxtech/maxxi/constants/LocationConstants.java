package com.maxtech.maxxi.constants;

import static com.maxtech.maxxi.constants.VisionConstants.*;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public final class LocationConstants {
    public static final double FIELD_WIDTH = Units.feetToMeters(27.0);
    public static final double FIELD_LENGTH = Units.feetToMeters(54.0);
    public static final double FIELD_MIDDLE = Units.feetToMeters(54.0) / 2.0;

    // PLacing positions
    private static final double CONE_X_OFFSET = 0.5;
    private static final double RED_GRID_X = 2.0;
    private static final double BLUE_GRID_X = 14.5;
    public static final RobotPositionConstants RED_GRID_FAR = new RobotPositionConstants(TAG1, new Pose2d(RED_GRID_X, TAG1.pose.getY(), TAG1.pose.getRotation().toRotation2d()));
    public static final RobotPositionConstants RED_GRID_CENTER = new RobotPositionConstants(TAG2, new Pose2d(RED_GRID_X, TAG2.pose.getY(), TAG2.pose.getRotation().toRotation2d()));
    public static final RobotPositionConstants RED_GRID_PICKUP = new RobotPositionConstants(TAG3, new Pose2d(RED_GRID_X, TAG3.pose.getY(), TAG3.pose.getRotation().toRotation2d()));
    public static final RobotPositionConstants BLUE_GRID_FAR = new RobotPositionConstants(TAG8, new Pose2d(BLUE_GRID_X, TAG8.pose.getY(), TAG8.pose.getRotation().toRotation2d()));
    public static final RobotPositionConstants BLUE_GRID_CENTER = new RobotPositionConstants(TAG7, new Pose2d(BLUE_GRID_X, TAG7.pose.getY(), TAG7.pose.getRotation().toRotation2d()));
    public static final RobotPositionConstants BLUE_GRID_PICKUP = new RobotPositionConstants(TAG6, new Pose2d(BLUE_GRID_X, TAG6.pose.getY(), TAG6.pose.getRotation().toRotation2d()));

    // Loading station positions
    public static final RobotPositionConstants RED_LOADING_STATION = new RobotPositionConstants(TAG4, new Pose2d(RED_GRID_X, TAG4.pose.getY(), TAG4.pose.getRotation().toRotation2d()));
    public static final RobotPositionConstants BLUE_LOADING_STATION = new RobotPositionConstants(TAG5, new Pose2d(BLUE_GRID_X, TAG5.pose.getY(), TAG5.pose.getRotation().toRotation2d()));

    public static void getClosetGrid(Pose2d robotPose) {
        /* Chose the closest grid to the robot to place. */
        if (robotPose.getTranslation().getX() <  FIELD_MIDDLE) {
            // Red side
            if (robotPose.getTranslation().getY() < 0.0) {
                // Far
                robotPose = RED_GRID_FAR.pose;
            } else if (robotPose.getTranslation().getY() < 0.0) {
                // Center
                robotPose = RED_GRID_CENTER.pose;
            } else {
                // Pickup
                robotPose = RED_GRID_PICKUP.pose;
            }
        } else {
            // Blue side
            if (robotPose.getTranslation().getX() > FIELD_MIDDLE) {
                // Far
                robotPose = BLUE_GRID_FAR.pose;
            } else if (robotPose.getTranslation().getY() < 0.0) {
                // Center
                robotPose = BLUE_GRID_CENTER.pose;
            } else {
                // Pickup
                robotPose = BLUE_GRID_PICKUP.pose;
            }
        }
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

