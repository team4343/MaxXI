package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

public class AutoBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final OdometrySubsystem odometrySubsystem;

    public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, OdometrySubsystem odometrySubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.odometrySubsystem = odometrySubsystem;

        addRequirements(drivetrainSubsystem, odometrySubsystem);
    }

    @Override
    public void execute() {

    }
}
