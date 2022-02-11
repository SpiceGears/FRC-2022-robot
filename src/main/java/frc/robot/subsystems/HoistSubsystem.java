// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class HoistSubsystem extends SubsystemBase {
  /** Creates a new Hoist. */

  private VictorSP rightMotor, leftMotor;
  private MotorControllerGroup hoistMotors;

  public HoistSubsystem() {
    rightMotor = new VictorSP(PortMap.Hoist.RIGHT_MOTOR);
    leftMotor = new VictorSP(PortMap.Hoist.LEFT_MOTOR);

    configureMotors();
  }

  private void configureMotors() {
    rightMotor.setInverted(false);
    leftMotor.setInverted(false);

    hoistMotors = new MotorControllerGroup(rightMotor, leftMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentageMotorOut(double out, double back) {
    hoistMotors.set(out - back);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Hoist percentage out", hoistMotors.get());
  }

}
