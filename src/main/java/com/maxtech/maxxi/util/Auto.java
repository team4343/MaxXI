package com.maxtech.maxxi.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto {
    public String name;
    public SequentialCommandGroup command = new SequentialCommandGroup();

    public Auto(String name, Command command) {
        this.name = name;
        this.command.addCommands(command);
    }

    public Auto(String name) {
        this.name = name;
    }

    public Auto addCommand(Command command) {
        this.command.addCommands(command);
        return this;
    }
}
