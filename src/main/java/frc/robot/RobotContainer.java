// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.IntakeSetCommand;
import frc.robot.commands.SuppliedDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.State;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

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
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        /*
         * m_drivetrainSubsystem.setDefaultCommand(new SuppliedDriveCommand(m_drivetrainSubsystem,
         * () -> -modifyAxis(m_controller.getLeftY()) DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
         * () -> -modifyAxis(m_controller.getLeftX()) DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
         * () -> -modifyAxis(m_controller.getRightX())
         * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
         */

        m_drivetrainSubsystem.setDefaultCommand(new SuppliedDriveCommand(m_drivetrainSubsystem, hid::getDriverX, hid::getDriverY, hid::getDriverT));

        // ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(),
        // m_drivetrainSubsystem.getGyroscopeRotation());

        // Configure the button bindings
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
        hid.setDriverCommand(1).onTrue(new InstantCommand(m_drivetrainSubsystem.gyroscope::reset, m_drivetrainSubsystem));
//        driver.setCommand(2).whileTrue(new AutoBalanceCommand(m_drivetrainSubsystem));
        hid.setDriverCommand(3).onTrue(new ArmPositionCommand(m_armSubsystem, State.Pickup));
        hid.setDriverCommand(5).onTrue(new ArmPositionCommand(m_armSubsystem, State.Rest));
        hid.setDriverCommand(4).onTrue(new ArmPositionCommand(m_armSubsystem, State.PlacingA));
        hid.setDriverCommand(6).onTrue(new ArmPositionCommand(m_armSubsystem, State.PlacingB));
        hid.setDriverCommand(11).whileTrue(new IntakeSetCommand(m_intakeSubsystem, IntakeState.CUBE_IN))
                .onFalse(new IntakeSetCommand(m_intakeSubsystem, IntakeState.STOPPED));
        hid.setDriverCommand(12).onTrue(new IntakeSetCommand(m_intakeSubsystem, IntakeState.CUBE_OUT))
                .onFalse(new IntakeSetCommand(m_intakeSubsystem, IntakeState.STOPPED));

        hid.setDriverCommand(7).debounce(.1).onTrue(m_drivetrainSubsystem.moveOneMeterRight());

        // hid.setAssistantCommand(7).debounce(.1).onTrue(new GoToCommand(new PathPoint(new Translation2d(1, 1), Rotation2d.fromDegrees(-90)), m_drivetrainSubsystem));
        // hid.setAssistantCommand(8).debounce(.1).onTrue(new GoToCommand(new PathPoint(new Translation2d(1, 2), Rotation2d.fromDegrees(-90)), m_drivetrainSubsystem));
        // hid.setAssistantCommand(9).debounce(.1).onTrue(new GoToCommand(new PathPoint(new Translation2d(1, 3), Rotation2d.fromDegrees(-90)), m_drivetrainSubsystem));


        // Operator
//        operator.setCommand(5).onTrue(new InstantCommand(() -> m_armSubsystem.setState(State.Rest)));
//        operator.setCommand(7).onTrue(new InstantCommand(() -> m_armSubsystem.setState(State.PlacingA)));
//        operator.setCommand(8).onTrue(new InstantCommand(() -> m_armSubsystem.setState(State.PlacingB)));
//        operator.setCommand(9).onTrue(new InstantCommand(() -> m_armSubsystem.setState(State.PlacingC)));
//        operator.setCommand(10).onTrue(new InstantCommand(() -> m_armSubsystem.setState(State.PlacingD)));
//        operator.setCommand(11).onTrue(new InstantCommand(() -> m_armSubsystem.setState(State.PlacingE)));

        // Station Pickup
//        operator.setCommand(1).onTrue(
//                new ParallelCommandGroup(
//                        new AlignCommand(m_drivetrainSubsystem, new AprilTag[]{AprilTags.tag03}),
//                        new InstantCommand(() -> m_armSubsystem.setState(State.PlacingC)),
//                        new InstantCommand(() -> m_intakeSubsystem.setState(IntakeState.CUBE_IN))
//                ).andThen(
//                        new InstantCommand(() -> m_armSubsystem.setState(State.Rest)),
//                        new InstantCommand(() -> m_intakeSubsystem.setState(IntakeState.STOPPED))
//                )
//        );
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

    public static class HID {
        private final Joystick m_driverStick;
        private final Joystick m_assistantStick;

        public HID(int driverPort, int assistantPort) {
            m_driverStick = new Joystick(driverPort);
            m_assistantStick = new Joystick(assistantPort);
//            var tab = Shuffleboard.getTab("HID");
//            tab.addNumber("X", this::getX);
//            tab.addNumber("Y", this::getY);
//            tab.addNumber("T", this::getT);
        }

        /*
         * Get the joystick's X value in meters per second.
         * 
         * Note that the joystick's Y is the robot's X.
         * @return the joystick's X value in meters per second.
         */
        public double getDriverX() {
            return -modifyAxis(m_driverStick.getY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        }

        /*
         * Get the joystick's Y value in meters per second.
         * 
         * Note that the joystick's X is the robot's Y.
         * @return the joystick's Y value in meters per second.
         */
        public double getDriverY() {
            return -modifyAxis(m_driverStick.getX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        }

        /*
         * Get the joystick's rotational value in radians per second.
         *
         * @return the joystick's rotational value in radians per second.
         */
        public double getDriverT() {
            return modifyAxis(m_driverStick.getTwist()) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        }

        public JoystickButton setDriverCommand(int button) {
            return new JoystickButton(m_driverStick, button);
        }

        public JoystickButton setAssistantCommand(int button) {
            return new JoystickButton(m_assistantStick, button);
        }

        private double deadband(double value, double deadband) {
            if (Math.abs(value) > deadband) {
                if (value > 0.0) {
                    return (value - deadband) / (1.0 - deadband);
                } else {
                    return (value + deadband) / (1.0 - deadband);
                }
            } else {
                return 0.0;
            }
        }

        private double modifyAxis(double value) {
            // Deadband
            value = deadband(value, 0.1);

            // Square the axis
            value = Math.copySign(value * value, value);

            return value;
        }
    }
}
