package frc.robot.constants;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import static java.lang.Math.PI;

public final class DriveConstants {
    // How much do we trust our odometry inputs. 0.0 is no trust, 1.0 is full trust.
    public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5.0));
    public static final Vector<N3> visionStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    public static final PIDController xPIDController = new PIDController(0.6, 0.01, 0.0);
    public static final PIDController yPIDController = new PIDController(0.6, 0.01, 0.0);
    public static final PIDController rPIDController = new PIDController(0.05, 0.005, 0.0);

    public static final TrapezoidProfile.Constraints rTrapezoidConstraints = new TrapezoidProfile.Constraints(PI, 2 / PI);
    public static final ProfiledPIDController rTrapezoidPIDController = new ProfiledPIDController(6.0, 0.02, 0.0, rTrapezoidConstraints);

    public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.56;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.56;
    public static final double MAX_VOLTAGE = 11.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * PI; // SDS recommends 5676 RPM to M/S
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 6.0; // SDS recommends 6 rad/s

    public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(50.0 + 90.0 + 5.0 + 180);
    public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(360.0-70.0-40.0 - 180);
    public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(360.0-30.0 - 180);
    public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(45.0+ 30.0 + 180);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        // Positive X values are towards center of the field.
        // Positive Y values are towards the left hand side of the robot.
        // All kinematics and swerve modules should be referenced in this order.
        new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
        new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
        new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
        new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
    );


}
