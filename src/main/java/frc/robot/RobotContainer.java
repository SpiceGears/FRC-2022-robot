// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final DriveTrainSubsystem m_DriveTrainSubsystem = new DriveTrainSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  XboxController m_driverController = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_DriveTrainSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              m_DriveTrainSubsystem.setMotorPercentageOutput(
                  m_driverController.getRawAxis(1),
                  m_driverController.getRawAxis(1),
                  m_driverController.getRawAxis(2) * Constants.Joystick.JOYSTICK_TURN_AXIS_MULTIPLIER);
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

    button_a.whenPressed(new ToggleIntake());
  }

  // /**
  // * Use this to pass the autonomous command to the main {@link Robot} class.
  // *
  // * @return the command to run in autonomous
  // */
  // public Command getAutonomousCommand() {
  // // An ExampleCommand will run in autonomous
  // return m_autoCommand;
  // }

  public void updateSmartDashboard() {
    m_DriveTrainSubsystem.updateSmartDashboard();
    SmartDashboard.putNumber("joistick x", m_driverController.getRawAxis(1));
  }
}
