package com.maxtech.maxxi.commands.drivebase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.maxtech.lib.swervelib.math.SwerveModuleState2;

public class DriveToPoint extends CommandBase {
    private final Pose2d pose;

    private final DrivetrainSubsystem drivetrainSubsystem;

    private boolean finished = false;

    public DriveToPoint(Pose2d pose, DrivetrainSubsystem drivetrainSubsystem) {
        this.pose = pose;

        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        var currentPose = drivetrainSubsystem.getPose();
        var goalPose = pose.relativeTo(currentPose);

        List<Pose2d> waypoints = Arrays.asList(currentPose, goalPose);

        var trajectory =
                TrajectoryGenerator.generateTrajectory(waypoints, new TrajectoryConfig(1, .1));

        var controller = new HolonomicDriveController(new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, .1)));

        var totalTime = trajectory.getTotalTimeSeconds();

        var startTime = Timer.getFPGATimestamp();

        // TODO: double check that this for method is right.
        for (double time = Timer.getFPGATimestamp(); (time - startTime) <= totalTime; time = Timer.getFPGATimestamp()) {
            // TODO: we may have to round the FPGATimestamp because it's too precise.
            var sample = trajectory.sample(Timer.getFPGATimestamp());

            // TODO: what's the rotation argument?
            ChassisSpeeds adjustedSpeeds = controller.calculate(drivetrainSubsystem.getPose(),
                    sample, sample.poseMeters.getRotation());

            SwerveModuleState2[] moduleStates =
                    drivetrainSubsystem.getKinematics().toSwerveModuleStates(adjustedSpeeds);

            drivetrainSubsystem.setModuleStates(moduleStates);
        }

        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
