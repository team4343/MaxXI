package com.maxtech.maxxi.commands.drivebase;

import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToPoint extends CommandBase {
    private final Trajectory trajectory;
    
    private final DrivetrainSubsystem drivetrainSubsystem;

    public DriveToPoint(Trajectory trajectory, DrivetrainSubsystem drivetrainSubsystem) {
        this.trajectory = trajectory;

        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        var states = trajectory.getStates();

        states.forEach(state -> {
            var relativePose = state.poseMeters.relativeTo(drivetrainSubsystem.getPose());

            // Here, we have a robot-relative pose in meters, we just have to drive the bot to that point.
        });
    }
}
