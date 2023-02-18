package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetCommand extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;
    private final IntakeSubsystem.IntakeState m_state;

    public IntakeSetCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeState state) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeSubsystem);
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_state = state;
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intakeSubsystem.setState(m_state);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
