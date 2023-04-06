package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.LightSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/*
 * Automatically set the LED colors (yellow/blue) based on NT input. Scheduled
 * in @{Robot.java}.
 */
public class AutoLEDSetCommand extends CommandBase {
    private final LightSubsystem m_lightSubsystem = LightSubsystem.getInstance();
    private final SendableChooser<String> ledChooser;

    public AutoLEDSetCommand(SendableChooser<String> ledChooser) {
        this.ledChooser = ledChooser;

        addRequirements(m_lightSubsystem);
    }

    @Override
    public void execute() {
        // Periodically, we get the status of the lights desired and set
        // accordingly.
        var ledChoice = ledChooser.getSelected();
        if (ledChoice.equals("Cone")) {
            System.out.println("Setting to cone.");
            m_lightSubsystem.setState(LightSubsystem.State.Yellow);
        } else if (ledChoice.equals("Cube")) {
            m_lightSubsystem.setState(LightSubsystem.State.Purple);
        } else {
            m_lightSubsystem.setState(LightSubsystem.State.Blue);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
