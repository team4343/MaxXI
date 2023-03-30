// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.IntakeSetCommand;
import frc.robot.commands.SuppliedDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.State;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.OdometrySubsystem;

import frc.robot.util.HumanDevice;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final HumanDevice hid = new HumanDevice(0, 1);
    public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final OdometrySubsystem odometrySubsystem = new OdometrySubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrainSubsystem.setDefaultCommand(
            new SuppliedDriveCommand(drivetrainSubsystem, odometrySubsystem,
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
        // hid.setDriverCommand(1).onTrue(new ResetOdometry(drivetrainSubsystem));
        hid.setDriverCommand(3).onTrue(new ArmPositionCommand(armSubsystem, State.PickupGround));
        hid.setDriverCommand(5).onTrue(new ArmPositionCommand(armSubsystem, State.Rest));
        hid.setDriverCommand(4).onTrue(new ArmPositionCommand(armSubsystem, State.PlacingMiddle));
        hid.setDriverCommand(6).onTrue(new ArmPositionCommand(armSubsystem, State.PLacingUpper));
        hid.setDriverCommand(2).onTrue(new ArmPositionCommand(armSubsystem, State.PickupStation));

        hid.setDriverCommand(11).whileTrue(new IntakeSetCommand(intakeSubsystem, IntakeState.CUBE_IN)).onFalse(new IntakeSetCommand(intakeSubsystem, IntakeState.STOPPED));
        hid.setDriverCommand(12).onTrue(new IntakeSetCommand(intakeSubsystem, IntakeState.CUBE_OUT)).onFalse(new IntakeSetCommand(intakeSubsystem, IntakeState.STOPPED));
    }

}