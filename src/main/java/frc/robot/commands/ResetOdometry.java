package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.ArrayList;

public class ResetOdometry extends CommandBase {
    DrivetrainSubsystem m_drivetrainSubsystem;
    PathPlannerTrajectory trajectory;
    boolean finished = false;
    double time_start = 0;

    public ResetOdometry(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        DrivetrainSubsystem.gyroscope.reset();

    }

    @Override
    public void execute() {
        if (DriverStation.getMatchTime() - 3 < this.time_start)
             this.finished = true;
        this.m_drivetrainSubsystem.gyroscope.reset();
        this.finished = true;
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
