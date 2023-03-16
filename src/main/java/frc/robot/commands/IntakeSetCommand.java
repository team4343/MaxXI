package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetCommand extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    private final IntakeSubsystem.IntakeState m_state;

    public IntakeSetCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeState state) {
        addRequirements(intakeSubsystem);
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_state = state;
    }

    @Override
    public void execute() {
        m_intakeSubsystem.setState(m_state);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_requirements.clear();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
