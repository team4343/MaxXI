// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class ArmConstants {
        public static final int SHOULDER_ID = 33;
        public static final int SHOULDER_FOLLOWER_ID = 30;
        public static final int ELBOW_ID = 32;
        public static final int INTAKE_ID = 31;
        public static final double STATE_EPSILON = 10;

        public static double ELBOW_GEAR_RATIO(double desired) {
            // return (desired / (2 * Math.PI)) * 70 * 40 / 22;
            return desired * 70 * 40 / 22;
        }

        public static double SHOULDER_GEAR_RATIO(double desired) {
            // return (desired / (2 * Math.PI)) * 40 * 60 / 16;
            return desired * 40 * 60 / 16;
        }
    }

    public static class DriveConstants {
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.56;
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.56;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11; // FIXME Set front left module
                                                                    // drive
                                                                    // motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10; // FIXME Set front left module
                                                                    // steer
                                                                    // motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 20; // FIXME Set front left steer
                                                                      // encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
                -Math.toRadians(328.44729544887844); // FIXME
                                                     // Measure
                                                     // and
                                                     // set
                                                     // front
                                                     // left
                                                     // steer
                                                     // offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13; // FIXME Set front right drive
                                                                     // motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12; // FIXME Set front right steer
                                                                     // motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 21; // FIXME Set front right
                                                                       // steer
                                                                       // encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
                -Math.toRadians(232.73437928862904); // FIXME Measure and set front right steer
                                                     // offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_OG = 15; // FIXME Set back left drive
                                                                      // motor
                                                                      // ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR_OG = 14; // FIXME Set back left steer
                                                                      // motor
                                                                      // ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 22; // FIXME Set back left steer
                                                                     // encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET =
                -Math.toRadians(162.86132551334515); // FIXME
                                                     // Measure
                                                     // and
                                                     // set
                                                     // back
                                                     // left
                                                     // steer
                                                     // offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 17; // FIXME Set back right drive
                                                                    // motor
                                                                    // ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 16; // FIXME Set back right steer
                                                                    // motor
                                                                    // ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23; // FIXME Set back right steer
                                                                      // encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
                -Math.toRadians(252.33398960257415); // FIXME
                                                     // Measure
                                                     // and
                                                     // set
                                                     // back
                                                     // right
                                                     // steer
                                                     // offset

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed.
         */
        public static final double MAX_VOLTAGE = 11.0;

        // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk4i standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND =
                5676.0 / 60.0 * SdsModuleConfigurations.MK4I_L1.getDriveReduction()
                        * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;

        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
                MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                // Front left
                new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Front right
                new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back left
                new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back right
                new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    }

    public static class FieldConstants {
        public static final double length = Units.feetToMeters(54); // 54
        public static final double width = Units.feetToMeters(27);
    }

    public static class VisionConstants {
        public static final Transform3d robotToCam = new Transform3d(
                new Translation3d(-.13, .30, .23), new Rotation3d(0, 0, Math.PI * 1.5)); // Cam
                                                                                         // mounted
                                                                                         // facing
                                                                                         // forward,
                                                                                         // half a
                                                                                         // meter
                                                                                         // forward
                                                                                         // of
                                                                                         // center,
                                                                                         // half a
                                                                                         // meter up
        // from center.
        public static final String cameraName = "OV5647";
    }

    /*
     * Given a current and desired value, return whether the values are in each other's range based
     * on a provided epsilon.
     */
    public static boolean goalCalculator(double current, double desired, double epsilon) {
        if (current == desired)
            return true;
        return Math.abs(current - desired) <= epsilon;
    }
}
