package com.maxtech.maxxi.commands.drivebase;

import static com.maxtech.maxxi.constants.DriveConstants.*;
import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory(DrivetrainSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(drivebase);
        if (resetOdometry) {
            drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
        }

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
