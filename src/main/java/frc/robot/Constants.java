// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.jsontype.impl.StdSubtypeResolver;

import edu.wpi.first.math.StateSpaceUtil;

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

        public static final double ROBOT_DRIVE_WHEEL_CIRCUIT = 478.77872; // in mm
        public static final double ENCODER_TICK_RATE = 256;
        public static final double ENCODER_MIN_RATE = 10; // Configures the encoder to consider itself stopped when it
                                                          // is bellow min rate
        public static final int ENCODER_SAMPLES_TO_AVERAGE = 5; // Can be between 1 and 127

        public static final boolean ENCODER_RIGHT_REVERSE = false;
        public static final boolean ENCODER_LEFT_REVERSE = true;

        public static final double ROBOT_DISTANCE_PER_ROTATION = ROBOT_DRIVE_WHEEL_CIRCUIT; // The distance the robot
                                                                                            // will travel with one
                                                                                            // revolution of the motor
                                                                                            // [mm]

        public static final double MOTOR_MIN_OUT = 0.27; // Configures the joystick to consider itself as still (0)
                                                         // when it is bellow min rate
    }

    public static class Intake {

        // NOT CURRENT VALUES
        public static final double ROBOT_INTAKE_WHEEL_CIRCUIT = 100; // [mm]
        public static final double ENCODER_TICK_RATE = 256;
        public static final double ENCODER_MIN_RATE = 10; // Configures the encoder to consider itself stopped when it is bellow min rate
        public static final double ROBOT_DISTANCE_PER_ROTATION = ROBOT_INTAKE_WHEEL_CIRCUIT; // The distance the robot will travel with one revolution of the motor [mm]
        
        public static final int ENCODER_SAMPLES_TO_AVERAGE = 5; // Can be between 1 and 127

        public static final boolean ENCODER_REVERSE = false;
        
    }
}
