package com.maxtech.maxxi.commands;


import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockDrive extends CommandBase {
    DrivetrainSubsystem drivetrainSubsystem;

    public LockDrive(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute() {
        this.drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
