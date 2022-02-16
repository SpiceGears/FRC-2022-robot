// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.Intake.SetIntakeOutput;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.HoistSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        public DriveTrainSubsystem m_DriveTrainSubsystem = new DriveTrainSubsystem();
        public Camera cameraSubsystem = new Camera();
        public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
        public HoistSubsystem m_HoistSubsystem = new HoistSubsystem();

        XboxController m_driverController = new XboxController(0);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                m_DriveTrainSubsystem.setDefaultCommand(
                                new RunCommand(() -> {
                                        m_DriveTrainSubsystem.joystickDriveTrain(
                                                        m_driverController.getRawAxis(1),
                                                        -m_driverController.getRawAxis(4)
                                                                        * Constants.Joystick.JOYSTICK_TURN_AXIS_MULTIPLIER);
                                }, m_DriveTrainSubsystem));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                final JoystickButton button_a = new JoystickButton(m_driverController, 1);
                final JoystickButton button_b = new JoystickButton(m_driverController, 2);

                button_a.whileActiveOnce(new ToggleIntake(m_IntakeSubsystem));
                button_b.whenPressed(new SetIntakeOutput(m_IntakeSubsystem));

                m_HoistSubsystem.setDefaultCommand(
                                new RunCommand(
                                                () -> {
                                                        m_HoistSubsystem.setPercentageMotorOut(
                                                                        m_driverController.getRawAxis(2),
                                                                        m_driverController.getRawAxis(3));
                                                }, m_HoistSubsystem));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An ExampleCommand will run in autonomous

                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(
                                                Constants.DriveTrain.Feedforward.KS,
                                                Constants.DriveTrain.Feedforward.KV,
                                                Constants.DriveTrain.Feedforward.KA),
                                Constants.DriveTrain.kDriveKinematics,
                                10);

                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.DriveTrain.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.DriveTrain.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(
                                                                Constants.DriveTrain.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, 0)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(2, 0, new Rotation2d(0)),
                                // Pass config
                                config);

                RamseteCommand ramseteCommand = new RamseteCommand(
                                exampleTrajectory,
                                m_DriveTrainSubsystem::getPose,
                                new RamseteController(Constants.DriveTrain.AutoConstants.kRamseteB,
                                                Constants.DriveTrain.AutoConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(
                                                Constants.DriveTrain.Feedforward.KS,
                                                Constants.DriveTrain.Feedforward.KV,
                                                Constants.DriveTrain.Feedforward.KA),
                                Constants.DriveTrain.kDriveKinematics,
                                m_DriveTrainSubsystem::getWheelSpeeds,
                                new PIDController(
                                                Constants.DriveTrain.PID.LEFT_KP,
                                                Constants.DriveTrain.PID.LEFT_KI,
                                                Constants.DriveTrain.PID.LEFT_KD),
                                new PIDController(
                                                Constants.DriveTrain.PID.RIGHT_KP,
                                                Constants.DriveTrain.PID.RIGHT_KI,
                                                Constants.DriveTrain.PID.RIGHT_KD),
                                // RamseteCommand passes volts to the callback
                                m_DriveTrainSubsystem::tankDriveVolts,
                                m_DriveTrainSubsystem);

                // Reset odometry to the starting pose of the trajectory.
                m_DriveTrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return ramseteCommand.andThen(() -> m_DriveTrainSubsystem.tankDriveVolts(0, 0));
        }

        public void updateSmartDashboard() {
                m_DriveTrainSubsystem.updateSmartDashboard();
                m_IntakeSubsystem.updateSmartDashboard();
                SmartDashboard.putNumber("joystick x", m_driverController.getRawAxis(1));
        }
}
