package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.constants.DriveConstants;
import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // Get these values in degrees.
        var c_pitch = m_drivetrainSubsystem.getPitch().getRadians();
        var c_roll = m_drivetrainSubsystem.getRoll().getRadians();

        var maximum_velocity = 1;

        // Create the rates, in meters per second.
        var c_xAxisRate =
                Math.sin(c_pitch) * maximum_velocity;
        var c_yAxisRate =
                Math.sin(c_roll) * maximum_velocity;

        // Drive in the opposite direction
        m_drivetrainSubsystem.drive(new Translation2d(-c_xAxisRate, -c_yAxisRate), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
