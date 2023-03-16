package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class SuppliedDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public SuppliedDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter tLimiter = new SlewRateLimiter(5);

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of //
        // field-oriented movement m_drivetrainSubsystem.drive(new ChassisSpeeds( //
        // m_drivetrainSubsystem.drive(new ChassisSpeeds(m_translationXSupplier.getAsDouble(),
        // m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble()));

        var x = xLimiter.calculate(m_translationXSupplier.getAsDouble());
        var y = yLimiter.calculate(m_translationYSupplier.getAsDouble());
        var t = tLimiter.calculate(m_rotationSupplier.getAsDouble());

        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x, y, t), DrivetrainSubsystem.gyroscope.getRotation2d()));
    }

    @Override
    public void end(boolean interrupted) {
//        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
