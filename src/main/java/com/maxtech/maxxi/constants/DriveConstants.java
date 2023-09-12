package com.maxtech.maxxi.constants;

import com.maxtech.swervelib.math.Matter;
import com.maxtech.swervelib.parser.PIDFConfig;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final PIDFConfig xAutoPIDConf = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPIDConf = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig rAutoPIDConf = new PIDFConfig(0.6, 0.01, 0.1);

    public static final double ROBOT_MASS = 135 * 0.453592; // 135lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.02 + 0.110; // 20ms + 110ms for the swerve controller
    public static final double DEAD_BAND = 0.25;

    public static final double MAX_ANGULAR_VELOCITY = 13; // SDS recommends 6 rad/s
    public static final double JOYSTICK_R_FACTOR = 0.70;
    public static final double SLOW_MODE_FACTOR = 0.50;

}
