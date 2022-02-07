// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class HoistSubsystem extends SubsystemBase {
  /** Creates a new Hoist. */

  private VictorSP rightMotor;

  public HoistSubsystem() {
    rightMotor = new VictorSP(PortMap.Hoist.RIGHT_MOTOR);

    configureMotors();
  }

  private void configureMotors() {
    rightMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentageMotorOut(double lOut, double rOut) {
    rightMotor.set(rOut);
  }

}
