package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.IntakeSubsystem;
import com.maxtech.maxxi.subsystems.LightSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class IntakeSetCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier speedSupplier;
    private final LightSubsystem ledSubsystem = LightSubsystem.getInstance();

    public IntakeSetCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier speed) {
        addRequirements(intakeSubsystem, ledSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.speedSupplier = speed;
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        intakeSubsystem.setSpeed(speed);

        if (intakeSubsystem.getCurrent() > 10) {
            ledSubsystem.setState(LightSubsystem.State.GreenBlinking);
            return;
        }

        if (speed > 0) {
            ledSubsystem.setState(LightSubsystem.State.Purple);
        } else if (speed < 0) {
            ledSubsystem.setState(LightSubsystem.State.Yellow);
        } else {
            LightSubsystem.getInstance().setState(DriverStation.Alliance.Blue == DriverStation.getAlliance() ? LightSubsystem.State.Blue : LightSubsystem.State.Red);
        }

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
