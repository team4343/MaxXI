// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import frc.robot.constants.LocationConstants;
import frc.robot.constants.RobotPositionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.State;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.util.HID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final HID hid = new HID(0, 1);

    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_drivetrainSubsystem.setDefaultCommand(
                new SuppliedDriveCommand(m_drivetrainSubsystem,
                        hid::getDriverX,
                        hid::getDriverY,
                        hid::getDriverT));
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
        hid.setDriverCommand(1).onTrue(new InstantCommand(DrivetrainSubsystem.gyroscope::reset, m_drivetrainSubsystem));
        hid.setDriverCommand(2).whileTrue(new AutoBalanceCommand(m_drivetrainSubsystem));
        hid.setDriverCommand(3).onTrue(new ArmPositionCommand(m_armSubsystem, State.Pickup));
        hid.setDriverCommand(5).onTrue(new ArmPositionCommand(m_armSubsystem, State.Rest));
        hid.setDriverCommand(4).onTrue(new ArmPositionCommand(m_armSubsystem, State.PlacingA));
        hid.setDriverCommand(6).onTrue(new ArmPositionCommand(m_armSubsystem, State.PlacingB));
        hid.setDriverCommand(11).whileTrue(new IntakeSetCommand(m_intakeSubsystem, IntakeState.CUBE_IN)).onFalse(new IntakeSetCommand(m_intakeSubsystem, IntakeState.STOPPED));
        hid.setDriverCommand(12).onTrue(new IntakeSetCommand(m_intakeSubsystem, IntakeState.CUBE_OUT)).onFalse(new IntakeSetCommand(m_intakeSubsystem, IntakeState.STOPPED));

        hid.setDriverCommand(7).onTrue(
                new ParallelCommandGroup(
                        new AlignCommand(m_drivetrainSubsystem, new RobotPositionConstants[]{LocationConstants.kRedLoadingStation, LocationConstants.kBlueLoadingStation}),
                        new InstantCommand(() -> m_armSubsystem.setState(State.PlacingC)),
                        new InstantCommand(() -> m_intakeSubsystem.setState(IntakeState.CUBE_IN))
                ).andThen(
                        new InstantCommand(() -> m_armSubsystem.setState(State.Rest)),
                        new InstantCommand(() -> m_intakeSubsystem.setState(IntakeState.STOPPED))
                )
        );
        // hid.setOperatorCommand(7).debounce(.1).onTrue(new GoToCommand(new PathPoint(new Translation2d(1, 1), Rotation2d.fromDegrees(-90)), m_drivetrainSubsystem));
        // hid.setOperatorCommand(8).debounce(.1).onTrue(new GoToCommand(new PathPoint(new Translation2d(1, 2), Rotation2d.fromDegrees(-90)), m_drivetrainSubsystem));
        // hid.setOperatorCommand(9).debounce(.1).onTrue(new GoToCommand(new PathPoint(new Translation2d(1, 3), Rotation2d.fromDegrees(-90)), m_drivetrainSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_drivetrainSubsystem.constructLiveTrajectoryCommand(m_armSubsystem);
    }

    public Command getArmCommand(State state) {
        return new ArmPositionCommand(m_armSubsystem, state);
    }

}
