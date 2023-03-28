package com.maxtech.maxxi.constants;

import com.maxtech.swervelib.math.Matter;
import com.maxtech.swervelib.parser.PIDFConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    // How much do we trust our odometry inputs. 0.0 is no trust, 1.0 is full trust.
    public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5.0));
    public static final Vector<N3> visionStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    public static final PIDController xPIDController = new PIDController(0.6, 0.01, 0.0);
    public static final PIDController yPIDController = new PIDController(0.6, 0.01, 0.0);
    public static final PIDController rPIDController = new PIDController(0.05, 0.005, 0.0);

    public static final PIDFConfig xAutoPIDConf = new PIDFConfig(0.4, 0, 0);
    public static final PIDFConfig yAutoPIDConf = new PIDFConfig(0.4, 0, 0);
    public static final PIDFConfig rAutoPIDConf = new PIDFConfig(0.2, 0, 0.01);

    public static final PIDController xBalanceController = new PIDController(0.6, 0.01, 0.0);
    public static final PIDController yBalanceController = new PIDController(0.6, 0.01, 0.0);
    public static final PIDController rBalanceController = new PIDController(0.05, 0.005, 0.0);

    public static final double X_LIMITER_MANUAL = 1.5;
    public static final double Y_LIMITER_MANUAL = 1.5;
    public static final double R_LIMITER_MANUAL = 3.0;

    public static final double ROBOT_MASS = 135 * 0.453592; // 135lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.02 + 0.110; // 20ms + 110ms for the swerve controller
    public static final double DEAD_BAND = 0.05;

    public static final double MAX_ANGULAR_VELOCITY = 6; // SDS recommends 6 rad/s
    public static final double SLOW_MODE_FACTOR = 0.35;

}
