package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

import java.util.function.DoubleSupplier;

public class SuppliedDriveCommand extends CommandBase {
    /*
     * This command is used to drive the robot using the supplied values.
     *
     * The values are supplied by the joystick, and are filtered to prevent sudden changes in speed.
     */

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final OdometrySubsystem od;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public SuppliedDriveCommand(DrivetrainSubsystem drivetrainSubsystem, OdometrySubsystem od,
                                DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
                                DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.od = od;

        addRequirements(drivetrainSubsystem);
    }

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter tLimiter = new SlewRateLimiter(5);

    @Override
    public void execute() {
        var x = xLimiter.calculate(translationXSupplier.getAsDouble());
        var y = yLimiter.calculate(translationYSupplier.getAsDouble());
        var t = tLimiter.calculate(rotationSupplier.getAsDouble());

        drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x, y, t), od.getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {}
}
