// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.maxtech.maxxi;

import com.maxtech.maxxi.commands.*;
import com.maxtech.maxxi.subsystems.ArmSubsystem;
import com.maxtech.maxxi.subsystems.ArmSubsystem.State;
import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import com.maxtech.maxxi.subsystems.IntakeSubsystem;
import com.maxtech.maxxi.util.HumanDevice;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final HumanDevice hid = new HumanDevice(1, 0, 2);

    public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrainSubsystem.setDefaultCommand(getAbsoluteFieldDriveCommand());
        intakeSubsystem.setDefaultCommand(new IntakeSetCommand(
            intakeSubsystem,
//            () -> hid.getOperatorTriggerL() - hid.getOperatorTriggerR()
            () -> hid.getPlaystationTriggerL() - hid.getPlaystationTriggerR()
        ));
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Driver
        hid.setDriverCommand(3).onTrue(new ArmPositionCommand(armSubsystem, State.PickupGround));
        hid.setDriverCommand(5).onTrue(new ArmPositionCommand(armSubsystem, State.Rest));
        hid.setDriverCommand(4).onTrue(new ArmPositionCommand(armSubsystem, State.PlacingMiddle));
        hid.setDriverCommand(6).onTrue(new ArmPositionCommand(armSubsystem, State.PLacingUpper));
        hid.setDriverCommand(2).onTrue(new ArmPositionCommand(armSubsystem, State.PickupStation));

        hid.setOperatorCommand(1).onTrue(new ArmPositionCommand(armSubsystem, State.PickupGround));
        hid.setOperatorCommand(2).onTrue(new ArmPositionCommand(armSubsystem, State.Rest));
        hid.setOperatorCommand(6).onTrue(new ArmPositionCommand(armSubsystem, State.PickupStation));
        hid.setOperatorCommand(3).onTrue(new ArmPositionCommand(armSubsystem, State.PlacingMiddle));
        hid.setOperatorCommand(4).onTrue(new ArmPositionCommand(armSubsystem, State.PLacingUpper));

        hid.setPlaystationCommand(2).onTrue(new ArmPositionCommand(armSubsystem, State.PickupGround));
        hid.setPlaystationCommand(3).onTrue(new ArmPositionCommand(armSubsystem, State.Rest));
        hid.setPlaystationCommand(6).onTrue(new ArmPositionCommand(armSubsystem, State.PickupStation));
        hid.setPlaystationCommand(1).onTrue(new ArmPositionCommand(armSubsystem, State.PlacingMiddle));
        hid.setPlaystationCommand(4).onTrue(new ArmPositionCommand(armSubsystem, State.PLacingUpper));

    }

    public Command getAbsoluteFieldDriveCommand() {
        return new AbsoluteFieldDrive(
            drivetrainSubsystem,
//            hid::getOperatorX,
//            hid::getOperatorY,
//            hid::getOperatorR,
            hid::getPlaystationX,
            hid::getPlaystationY,
            hid::getPlaystationR,
            false
        );
    }

    public Command getTeleopDriveCommand() {
        // Not needed
        return new TeleopDrive(
            drivetrainSubsystem,
            hid::getOperatorX,
            hid::getOperatorY,
            hid::getOperatorR,
            ()-> true,
            false,
            true
        );
    }

    public Command getAutonomousCommand() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(1, 0.5),
            new PathPoint(drivetrainSubsystem.getPose().getTranslation(), drivetrainSubsystem.getHeading(), drivetrainSubsystem.getHeading()),
            new PathPoint(drivetrainSubsystem.getPose().getTranslation(), drivetrainSubsystem.getHeading(), drivetrainSubsystem.getHeading())
        );

        return new FollowTrajectory(
            drivetrainSubsystem,
            trajectory,
            false
        );
    }


}