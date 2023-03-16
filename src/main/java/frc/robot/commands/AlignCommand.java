package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.ArrayList;

public class AlignCommand extends CommandBase {
    DrivetrainSubsystem m_drivetrainSubsystem;
    PathPlannerTrajectory trajectory;
    boolean finished = false;

    public AlignCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d pose = this.m_drivetrainSubsystem.odometry.getPose();
        Pose2d next = new Pose2d(2.0, 7.0, Rotation2d.fromRadians(Math.PI));
        ArrayList<PathPoint> points = new ArrayList<>();
        points.add(new PathPoint(pose.getTranslation(), pose.getRotation()));
        points.add(new PathPoint(next.getTranslation(), next.getRotation()));
        System.out.println(pose);
        System.out.println(next);
        this.trajectory = PathPlanner.generatePath(new PathConstraints(1, .2), points);
        this.m_drivetrainSubsystem.followTrajectoryCommand(this.trajectory).schedule();
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
