package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.LightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDSetCommand extends CommandBase {
    private final LightSubsystem ledSubsystem = LightSubsystem.getInstance();
    private final LightSubsystem.State state;
    private double blinkTime = 0;

    public LEDSetCommand(LightSubsystem.State state) {
        this.state = state;
        addRequirements(ledSubsystem);
    }

    public LEDSetCommand(LightSubsystem.State state, double blinkTime) {
        this.state = state;
        this.blinkTime = blinkTime;
        addRequirements(ledSubsystem);
    }

    public void execute() {
        ledSubsystem.setState(state);
        if (blinkTime != 0)
            ledSubsystem.setBlinkTime(blinkTime);
    }

    public void end(boolean interrupted) {
        this.m_requirements.clear();
    }

    public boolean isFinished() {
        return true;
    }
}
