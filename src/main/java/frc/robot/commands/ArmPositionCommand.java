package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final ArmSubsystem.State state;

    public ArmPositionCommand(ArmSubsystem armSubsystem, ArmSubsystem.State state) {
        this.armSubsystem = armSubsystem;
        this.state = state;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.setState(state);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_requirements.clear();
    }

    @Override
    public boolean isFinished() {
        // Only needs to set the state once.
        return true;
    }
}
