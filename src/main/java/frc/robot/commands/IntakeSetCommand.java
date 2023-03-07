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


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_intakeSubsystem.hasGamePiece())
            m_intakeSubsystem.setState(IntakeSubsystem.IntakeState.STOPPED);
        else
            m_intakeSubsystem.setState(m_state);
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
