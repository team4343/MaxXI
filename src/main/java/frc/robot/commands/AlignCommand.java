package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

import java.util.ArrayList;

public class AlignCommand extends CommandBase {
    /*
     * This command is used to align the robot to a specific location using the path planner.
     *
     * Execute override is not needed.
     * TODO - Implement this.
     */

    OdometrySubsystem odometrySubsystem;
    DrivetrainSubsystem drivetrainSubsystem;
    PathPlannerTrajectory trajectory;
    double x,y,r;

    public AlignCommand(DrivetrainSubsystem drivetrainSubsystem, OdometrySubsystem odometrySubsystem, double x, double y, double r) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.x = x;
        this.y = y;
        this.r = r;

        addRequirements(drivetrainSubsystem, odometrySubsystem);
    }

    @Override
    public void initialize() {
        Pose2d pose = this.odometrySubsystem.getPose();
        Pose2d next = new Pose2d(this.x, this.y, Rotation2d.fromRadians(Math.PI));
        ArrayList<PathPoint> points = new ArrayList<>();
        points.add(new PathPoint(pose.getTranslation(), pose.getRotation()));
        points.add(new PathPoint(next.getTranslation(), next.getRotation()));
        System.out.println(pose);
        System.out.println(next);
        this.trajectory = PathPlanner.generatePath(new PathConstraints(1, .2), points);
//        this.drivetrainSubsystem.followTrajectoryCommand(this.trajectory).schedule();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.m_requirements.clear();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // this.drivetrainSubsystem.trajctoryFollower.isFinished();
        // TODO Implement this.
        return false;
    }
}
