// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class PortMap {

    public static final class DriveTrain {
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

    public static final class Intake {
        public static final int INTAKE_MOTOR = 4;
    }

    public static final class Hoist {
        public static final int RIGHT_MOTOR = 5;
        public static final int LEFT_MOTOR = 6;

        public static final int ARM_SOLENOID_FOREWORD = 2;
        public static final int ARM_SOLENOID_REVERSE = 3;

        public static final int HOOK_SOLENOID_FOREWORD = 4;
        public static final int HOOK_SOLENOID_REVERSE = 5;

        public static final int ENCODER_A = 4;
        public static final int ENCODER_B = 5;
    }
}
