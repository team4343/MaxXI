package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForCommand extends CommandBase {
    DrivetrainSubsystem m_drivetrainSubsystem;
    boolean finished = false;
    double startTime = 0;

    public DriveForCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        startTime = DriverStation.getMatchTime();
    }

    @Override
    public void execute() {
        if (DriverStation.getMatchTime() - 2 > startTime ) {
            this.finished = true;
            m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(0.0, 0.0, 0.0), DrivetrainSubsystem.gyroscope.getRotation2d()));
        } else {

        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(0.2, 0.0, 0.0), DrivetrainSubsystem.gyroscope.getRotation2d()));
        }
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.m_requirements.clear();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }


}
