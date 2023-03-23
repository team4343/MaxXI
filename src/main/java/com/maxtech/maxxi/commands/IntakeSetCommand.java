package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeSetCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeSubsystem.IntakeState state;

    public IntakeSetCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeState state) {
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.state = state;
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
        // Only needs to set the state once.
        return true;
    }
}