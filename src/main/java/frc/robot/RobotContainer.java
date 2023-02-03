// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.SuppliedDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ArmSubsystem.State;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final WorkBenchModerator m_workBenchModerator = new WorkBenchModerator();
    private final HID hid = new HID();

    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(m_workBenchModerator);
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

    // private final XboxController m_controller = new XboxController(1);

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

        m_drivetrainSubsystem.setDefaultCommand(
                new SuppliedDriveCommand(m_drivetrainSubsystem, hid::getX, hid::getY, hid::getT));

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
        /*
         * // Back button zeros the gyroscope new Button(m_controller::getBackButton) // No
         * requirements because we don't need to interrupt anything
         * .whenPressed(m_drivetrainSubsystem.gyroscope::reset);
         * 
         * new Button(m_controller::getYButton) .whenHeld(new
         * AutoBalanceCommand(m_drivetrainSubsystem));
         */
        hid.getGyroResetButton().onTrue(new InstantCommand(
                () -> m_drivetrainSubsystem.gyroscope.reset(), m_drivetrainSubsystem));
        hid.getAutoBalanceButton().whileTrue(new AutoBalanceCommand(m_drivetrainSubsystem));

        hid.getPickupButton().onTrue(new InstantCommand(
            () -> m_armSubsystem.setState(State.Pickup)
        ));
        hid.getRestButton().onTrue(new InstantCommand(
            () -> m_armSubsystem.setState(State.Rest)
        ));
        hid.getPlacingAButton().onTrue(new InstantCommand(
            () -> m_armSubsystem.setState(State.PlacingA)
        ));
        hid.getPlacingBButton().onTrue(new InstantCommand(
            () -> m_armSubsystem.setState(State.PlacingB)
        ));
        hid.getPlacingCButton().onTrue(new InstantCommand(
            () -> m_armSubsystem.setState(State.PlacingC)
        ));
        hid.getPlacingDButton().onTrue(new InstantCommand(
            () -> m_armSubsystem.setState(State.PlacingD)
        ));
        hid.getPlacingEButton().onTrue(new InstantCommand(
            () -> m_armSubsystem.setState(State.PlacingE)
        ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        var currentPose = m_drivetrainSubsystem.odometry.getPose();

        ArrayList<PathPoint> points = new ArrayList<>();
        points.add(new PathPoint(currentPose.getTranslation(), currentPose.getRotation()));
        points.add(new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)));

        PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(.5, .5), points);
        return m_drivetrainSubsystem.followTrajectoryCommand(traj, true);
    }

    public class WorkBenchModerator {
        private final boolean isBench;

        public WorkBenchModerator() {
            File benchFile = new File(Filesystem.getDeployDirectory(), "isBench");
            if (benchFile.exists()) {
                isBench = true;
            } else {
                isBench = false;
            }
        }

        public boolean isBench() {
            return isBench;
        }

        public boolean isReal() {
            return !isBench;
        }
    }

    public class HID {
        private final Joystick m_flightStick = new Joystick(0);

        public HID() {
            var tab = Shuffleboard.getTab("HID");
            tab.addNumber("X", this::getX);
            tab.addNumber("Y", this::getY);
            tab.addNumber("T", this::getT);
        }

        /*
         * Get the joystick's X value in meters per second.
         * 
         * Note that the joystick's Y is the robot's X.
         */
        public double getX() {
            return -modifyAxis(m_flightStick.getY())
                    * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        }

        /*
         * Get the joystick's Y value in meters per second.
         * 
         * Note that the joystick's X is the robot's Y.
         */
        public double getY() {
            return -modifyAxis(m_flightStick.getX())
                    * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        }

        /*
         * Get the joystick's rotational value in radians per second.
         */
        public double getT() {
            return -modifyAxis(m_flightStick.getTwist())
                    * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        }

        public JoystickButton getGyroResetButton() {
            return new JoystickButton(m_flightStick, 1);
        }

        public JoystickButton getAutoBalanceButton() {
            return new JoystickButton(m_flightStick, 2);
        }

        public JoystickButton getPickupButton() {
            return new JoystickButton(m_flightStick, 3);
        }

        public JoystickButton getRestButton() {
            return new JoystickButton(m_flightStick, 5);
        }

        public JoystickButton getPlacingAButton() {
            return new JoystickButton(m_flightStick, 7);
        } 

        public JoystickButton getPlacingBButton() {
            return new JoystickButton(m_flightStick, 8);
        } 

        public JoystickButton getPlacingCButton() {
            return new JoystickButton(m_flightStick, 9);
        } 

        public JoystickButton getPlacingDButton() {
            return new JoystickButton(m_flightStick, 10);
        } 

        public JoystickButton getPlacingEButton() {
            return new JoystickButton(m_flightStick, 11);
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
            value = deadband(value, 0.10);

            // Square the axis
            value = Math.copySign(value * value, value);

            return value;
        }
    }
}
