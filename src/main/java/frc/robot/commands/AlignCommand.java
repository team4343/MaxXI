package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotPositionConstant;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.List;

public class AlignCommand extends CommandBase {
    DrivetrainSubsystem m_drivetrainSubsystem;
    RobotPositionConstant[] positions;
    Double max_speed = 1.0;
    Double max_accel = 0.1;
    PathPlannerTrajectory trajectory;

    public AlignCommand(DrivetrainSubsystem drivetrainSubsystem, RobotPositionConstant[] positions) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.positions = positions;

        addRequirements(drivetrainSubsystem);
    }

    public AlignCommand(DrivetrainSubsystem drivetrainSubsystem, RobotPositionConstant[] positions, Double max_speed, Double max_accel) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.positions = positions;
        this.max_speed = max_speed;
        this.max_accel = max_accel;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        int best_tag_id = DrivetrainSubsystem.pcw.photonCamera.getLatestResult().getBestTarget().getFiducialId();
        Pose2d path_end = null;
        for (RobotPositionConstant position : this.positions)
            if (position.tag.ID == best_tag_id) {
                path_end = position.pose;
                break;
            }
        if (path_end == null) {
            this.end(false);
            return;
        }

        Pose2d currentPose = this.m_drivetrainSubsystem.odometry.getPose();
        PathPoint[] points = new PathPoint[]{
            new PathPoint(currentPose.getTranslation(), currentPose.getRotation()),
            new PathPoint(path_end.getTranslation(), path_end.getRotation()),
        };

        this.trajectory = PathPlanner.generatePath(new PathConstraints(this.max_speed, this.max_accel), List.of(points));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.m_drivetrainSubsystem.followTrajectorySuppliedCommand(() -> this.trajectory);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.m_requirements.clear();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
