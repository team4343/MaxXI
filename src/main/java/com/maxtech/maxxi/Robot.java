package com.maxtech.maxxi;

import com.maxtech.maxxi.subsystems.LightSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

import java.io.File;
import java.util.Objects;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private NetworkTableInstance nt_handle;
    public static final SendableChooser<String> autoChooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings
        robotContainer = new RobotContainer();
        nt_handle = NetworkTableInstance.getDefault();
        nt_handle.getEntry("/SmartDashboard/startingX").setDouble(14.75);
        nt_handle.getEntry("/SmartDashboard/startingY").setDouble(5.0);
        nt_handle.getEntry("/SmartDashboard/startingR").setDouble(0.0);

        autoChooser.setDefaultOption("Default Auto", "Default");
        for (File file: Objects.requireNonNull(new File(Filesystem.getDeployDirectory(), "pathplanner/").listFiles())) {
            String name = file.getName().replaceAll("[.][^.]+$", "");
            autoChooser.addOption(name, name);
        }
        SmartDashboard.putData("AutoChoices", autoChooser);

        // Set up logging.
        Logger.configureLoggingAndConfig(robotContainer, false);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
     * commands, running already-scheduled commands, removing finished or interrupted commands,
     * and running subsystem periodic() methods. This must be called from the robot's periodic
     * block in order for anything in the Command-based framework to work.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();

        LightSubsystem.getInstance().setState(DriverStation.Alliance.Blue == DriverStation.getAlliance() ? LightSubsystem.State.Blue : LightSubsystem.State.Red);
    }

    @Override
    public void disabledPeriodic() {
        String desiredAuto = nt_handle.getEntry("/autonomous").getString("default");

        double startingX = nt_handle.getEntry("/SmartDashboard/startingX").getDouble(14.75);
        double startingY = nt_handle.getEntry("/SmartDashboard/startingY").getDouble(5);
        double startingR = nt_handle.getEntry("/SmartDashboard/startingR").getDouble(0.0);

        nt_handle.getEntry("/SmartDashboard/startingConfirmRobot").setDoubleArray(new double[]{startingX, startingY, startingR});

        nt_handle.getEntry("/autonomous").setString(desiredAuto);

        nt_handle.getEntry("/autoConfirmSelection").setString(autoChooser.getSelected());

        if (autoChooser.getSelected().contains("Default") || autoChooser.getSelected().contains("default") || autoChooser.getSelected().contains("DEFAULT"))
            DriverStation.reportWarning("SOMEONE DID NOT SELECT AUTO", false);
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        SmartDashboard.putNumber("Auto Start X", robotContainer.drivetrainSubsystem.getPose().getTranslation().getX());
        SmartDashboard.putNumber("Auto Start Y", robotContainer.drivetrainSubsystem.getPose().getTranslation().getY());

//        robotContainer.drivetrainSubsystem.swerveDrive.swerveController.addSlewRateLimiters(new SlewRateLimiter(0.0), new SlewRateLimiter(0.0), new SlewRateLimiter(0.0));

        LightSubsystem.getInstance().setState(DriverStation.Alliance.Blue == DriverStation.getAlliance() ? LightSubsystem.State.Blue : LightSubsystem.State.Red);

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(
            robotContainer.getAutonomousCommand(autoChooser.getSelected())
        );
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
//        robotContainer.drivetrainSubsystem.swerveDrive.swerveController.addSlewRateLimiters(xSlewRateLimiter, ySlewRateLimiter, rSlewRateLimiter);
        CommandScheduler.getInstance().cancelAll();
        LightSubsystem.getInstance().setState(DriverStation.Alliance.Blue == DriverStation.getAlliance() ? LightSubsystem.State.Blue : LightSubsystem.State.Red);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
