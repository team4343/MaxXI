package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCommand extends CommandBase {
    private final ArmSubsystem m_armSubsystem;
    private final ArmSubsystem.State m_state;

    public ArmPositionCommand(ArmSubsystem armSubsystem, ArmSubsystem.State state) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_armSubsystem = armSubsystem;
        this.m_state = state;
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_armSubsystem.setState(m_state);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
