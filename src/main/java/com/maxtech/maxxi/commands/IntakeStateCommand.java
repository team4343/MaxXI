package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeStateCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeSubsystem.State state;

    public IntakeStateCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.State state) {
        this.intakeSubsystem = intakeSubsystem;
        this.state = state;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setState(state);
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
