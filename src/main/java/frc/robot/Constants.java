// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Joystick {
        public static final double JOYSTICK_TURN_AXIS_MULTIPLIER = 0.3;
    }

    public static class DriveTrain {

        public static class PID {
            public static final double LEFT_KP = 8.6612E-05;
            public static final double LEFT_KI = 0;
            public static final double LEFT_KD = 0;

            public static final double RIGHT_KP = 8.5977E-05;
            public static final double RIGHT_KI = LEFT_KI;
            public static final double RIGHT_KD = LEFT_KD;
        }

        public static class Feedforward {
            public static final double KS = 1.6892;
            public static final double KV = 2.8577;
            public static final double KA = 0.0109;
        }

        public static class AutoConstants {

            public static double kMaxSpeedMetersPerSecond = 0.90886;
            public static double kMaxAccelerationMetersPerSecondSquared = 0.14696;

            // Reasonable baseline values for a RAMSETE follower in units of meters and
            // seconds
            public static final double kRamseteB = 2;
            public static final double kRamseteZeta = 0.7;

        }

        private static double kTrackWidthMeters = 590.0;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);

        public static final double ROBOT_DRIVE_WHEEL_CIRCUIT = 478.7787204; // in mm
        public static final double ENCODER_TICK_RATE = 2048;
        public static final double ENCODER_MIN_RATE = 10; // Configures the encoder to consider itself stopped when it
                                                          // is bellow min rate
        public static final int ENCODER_SAMPLES_TO_AVERAGE = 5; // Can be between 1 and 127

        public static final boolean ENCODER_RIGHT_REVERSE = false;
        public static final boolean ENCODER_LEFT_REVERSE = true;

        public static final double ROBOT_DISTANCE_PER_ROTATION = ROBOT_DRIVE_WHEEL_CIRCUIT; // The distance the robot
                                                                                            // will travel with one
                                                                                            // revolution of the motor
                                                                                            // [mm]

        public static final double MOTOR_MIN_PERCENTAGE_OUT = 0.2; // Configures the motor to consider itself as still
                                                                   // when it is bellow min rate
        public static final double MOTOR_MIN_VOLTAGE_OUT = 0.5; // Configures the motor to consider itself as still
                                                                // when it is bellow min rate
        public static final double MAX_ROBOT_SPEED = 12; // Max robot speed in teleoperated mode [m/s] (1m/s = 3,6km/h)
    }

}
