package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignCommand extends CommandBase {
    Pose2d start;
    DrivetrainSubsystem m_drivetrainSubsystem;
    AprilTag[] validTags;

    public AlignCommand(DrivetrainSubsystem drivetrainSubsystem, AprilTag[] validTags) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.validTags = validTags;
        addRequirements(drivetrainSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.m_drivetrainSubsystem.odometry.getPose();
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
