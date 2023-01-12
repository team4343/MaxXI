package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // Get these values in degrees.
        var c_pitch_deg = m_drivetrainSubsystem.getGyroscopePitch();
        var c_roll_deg = m_drivetrainSubsystem.getGyroscopeRoll();

        // Turn them into radians.
        var c_pitch_rad = c_pitch_deg * (Math.PI / 180.0);
        var c_roll_rad = c_roll_deg * (Math.PI / 180.0);

        // Create the rates, in meters per second.
        var c_xAxisRate = Math.sin(c_roll_rad) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 5;
        var c_yAxisRate = Math.sin(c_pitch_rad) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 5;

        // Drive in the opposite direction
        m_drivetrainSubsystem.drive(new ChassisSpeeds(c_xAxisRate, c_yAxisRate, 0));
    }
}
