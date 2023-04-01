// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.maxtech.maxxi;

import com.maxtech.maxxi.commands.*;
import com.maxtech.maxxi.subsystems.ArmSubsystem;
import com.maxtech.maxxi.subsystems.ArmSubsystem.State;
import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import com.maxtech.maxxi.subsystems.IntakeSubsystem;
import com.maxtech.maxxi.util.Auto;
import com.maxtech.maxxi.util.HumanDevice;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.ArrayList;

import static com.maxtech.maxxi.constants.LocationConstants.RED_GRID_CENTER;

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
    private final ArrayList<Auto> autos = new ArrayList<>();
    private final NetworkTableInstance nt_handle = NetworkTableInstance.getDefault();

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
        createAutos();
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

    public Command getAutonomousCommand(String auto) {
        double startingY = nt_handle.getEntry("/SmartDashboard/startingY").getDouble(0.0);
        double startingX = nt_handle.getEntry("/SmartDashboard/startingX").getDouble(0.0);
        double startingR = nt_handle.getEntry("/SmartDashboard/startingR").getDouble(0.0);
        double intakeSpeed = nt_handle.getEntry("/SmartDashboard/startingCube").getBoolean(false) ? 0.6 : -0.6;

        drivetrainSubsystem.resetOdometry(new Pose2d(startingX, startingY, Rotation2d.fromDegrees(startingR)));

        PathPlannerTrajectory trajectory = PathPlanner.loadPath(auto, new PathConstraints(1, 0.25), false);

        // Default to place the cone or cube
        SequentialCommandGroup autoCommands = new SequentialCommandGroup(
            new ArmPositionCommand(armSubsystem, State.PLacingUpper),
            new ArmPositionCommand(armSubsystem, State.PLacingUpper),
            new ArmPositionCommand(armSubsystem, State.PLacingUpper),
            new WaitCommand(2),
            new IntakeSetCommand(intakeSubsystem, intakeSpeed).withTimeout(0.75),
            new IntakeSetCommand(intakeSubsystem, 0.0).withTimeout(0.0),
            new ArmPositionCommand(armSubsystem, State.Rest)
        );

        switch (auto) {
            case "BlueCover":
            case "BlueOpen":
            case "RedCover":
            case "RedOpen":
                autoCommands.addCommands(
                    new FollowTrajectory(drivetrainSubsystem, trajectory, true),
                    new PointRotate(drivetrainSubsystem, drivetrainSubsystem.getHeading().plus(Rotation2d.fromDegrees(180))),
                    new ArmPositionCommand(armSubsystem, State.PickupGround),
                    new IntakeStateCommand(intakeSubsystem, IntakeSubsystem.State.ConeIn)
                );
                return autoCommands;
            case "BluePlatform":
            case "RedPlatform":
                autoCommands.addCommands(
                    new FollowTrajectory(drivetrainSubsystem, trajectory, true)
                );
                return autoCommands;
        }

        DriverStation.reportError("SOMEONE DID NOT SELECT AUTO", false);

        // Defualt to place the cone.
        return autoCommands;


    }

    private void createAutos() {
        this.autos.add(new Auto("Default")); // Default Do Nothing
        this.autos.add(new Auto("Red 1", new DriveToPoint(drivetrainSubsystem, RED_GRID_CENTER.pose))); // Drive to center grid pose.
    }

    public Auto matchAuto(String autoName) {
        for (Auto auto : this.autos)
            if (auto.name.equals(autoName))
                return auto;

        return new Auto("Errored Out On matchAuto()");
    }


}