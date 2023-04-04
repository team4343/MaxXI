package com.maxtech.maxxi.commands;

import com.maxtech.maxxi.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Auto Balance command using a simple PID controller.
 *
 * Note that tilting the robot from the back changes the pitch.
 */
public class AutoBalanceCommand extends CommandBase {

    private final DrivetrainSubsystem DrivetrainSubsystem;
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;

    public AutoBalanceCommand(DrivetrainSubsystem DrivetrainSubsystem) {
        this.DrivetrainSubsystem = DrivetrainSubsystem;
        xController = new ProfiledPIDController(.2, 0.0, 0.0, new Constraints(.4, .2));
        xController.setGoal(new State(0, 0));
        xController.setTolerance(1);

        yController = new ProfiledPIDController(.2, 0, 0, new Constraints(.4, .2));
        yController.setGoal(new State(0, 0));
        yController.setTolerance(1);

        addRequirements(this.DrivetrainSubsystem);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
     * called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        SmartDashboard.putBoolean("Auto balance at Tolerance",
                xController.atSetpoint() && yController.atSetpoint());

        // Get the pitch & roll of the bot, and calculate our next pitch and roll.
        double adjustedPitch = xController.calculate(DrivetrainSubsystem.getPitch().getDegrees());
        double adjustedRoll = yController.calculate(DrivetrainSubsystem.getRoll().getDegrees());

        SmartDashboard.putNumber("Auto balance adjusted pitch", adjustedPitch);
        SmartDashboard.putNumber("Auto balance adjusted roll", adjustedRoll);

        // For every degree of desired rotation, drive 1/x meters/second.
        // Current: at 45 degrees, drive 2 m/s.
        double drivableX = adjustedPitch / 22.5;
        double drivableY = adjustedRoll / 22.5;

        SmartDashboard.putNumber("Auto balance desired X", adjustedPitch);
        SmartDashboard.putNumber("Auto balance desired Y", adjustedRoll);

        DrivetrainSubsystem.drive(new Translation2d(-drivableX, drivableY), 0.0, false, false);
    }

    private double previousSuccessfulTime = 0;

    /**
     * The action to take when the command ends. Called when either the command finishes normally --
     * that is it is called when {@link #isFinished()} returns true -- or when it is
     * interrupted/canceled. This is where you may want to wrap up loose ends, like shutting off a
     * motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        DrivetrainSubsystem.lock();
    }
}
