package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.HumanDevice;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings
        robotContainer = new RobotContainer();

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
    public void disabledInit() {CommandScheduler.getInstance().cancelAll();}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        HumanDevice.alliance_modifier = DriverStation.getAlliance() == Alliance.Red ? -1 : 1;
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
