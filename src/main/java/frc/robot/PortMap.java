// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class PortMap {

    public static class DriveTrain {
        public static final int FRONT_LEFT_MOTOR = 3;
        public static final int BACK_LEFT_MOTOR = 2;

        public static final int FRONT_RIGHT_MOTOR = 1;
        public static final int BACK_RIGHT_MOTOR = 0;

        public static final int LEFT_ENCODER_A = 0;
        public static final int LEFT_ENCODER_B = 1;
        public static final int RIGHT_ENCODER_A = 2;
        public static final int RIGHT_ENCODER_B = 3;
        public static final int GYRO = 0;
    }

    public static class Intake {

        // NOT CURRENT VALUES
        public static final int INTAKE_MOTOR = 4;

        public static final DigitalSource INTAKE_ENCODER = null;
        public static final DigitalSource ENCODER_A = null;
        public static final DigitalSource ENCODER_B = null;
    }
}
