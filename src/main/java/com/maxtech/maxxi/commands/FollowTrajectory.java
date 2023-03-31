package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static com.maxtech.maxxi.constants.DriveConstants.*;


public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory(DrivetrainSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(drivebase);
        if (resetOdometry) {
            drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
        }

        addCommands(
            new PPSwerveControllerCommand(
                trajectory,
                drivebase::getPoseAuto,
                xAutoPIDConf.createPIDController(),
                yAutoPIDConf.createPIDController(),
                rAutoPIDConf.createPIDController(),
                drivebase::setChassisSpeedsAuto,
                drivebase),
            new StopDrive(drivebase)
        );
    }
}
