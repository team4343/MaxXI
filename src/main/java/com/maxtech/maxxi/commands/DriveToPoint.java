package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static com.maxtech.maxxi.constants.DriveConstants.*;

public class DriveToPoint extends SequentialCommandGroup {
    public DriveToPoint(DrivetrainSubsystem drivebase, Pose2d desiredPose) {
        addRequirements(drivebase);

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(1, 0.5),
            new PathPoint(drivebase.getPose().getTranslation(), drivebase.getHeading()),
            new PathPoint(desiredPose.getTranslation(), desiredPose.getRotation())
        );

        addCommands(
            new PPSwerveControllerCommand(
                trajectory,
                drivebase::getPose,
                xAutoPIDConf.createPIDController(),
                yAutoPIDConf.createPIDController(),
                rAutoPIDConf.createPIDController(),
                drivebase::setChassisSpeeds,
                drivebase)
        );
    }
}
