package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class IntakeSetCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier speedSupplier;

    public IntakeSetCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier speed) {
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.speedSupplier = speed;
    }

    @Override
    public void execute() {
        intakeSubsystem.setSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        this.m_requirements.clear();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
