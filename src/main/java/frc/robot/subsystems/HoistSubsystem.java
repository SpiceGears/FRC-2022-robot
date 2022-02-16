// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.concurrent.atomic.DoubleAccumulator;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class HoistSubsystem extends SubsystemBase {
  /** Creates a new Hoist. */

  private VictorSP rightMotor, leftMotor;
  private MotorControllerGroup hoistMotors;

  private Compressor compressor;

  public static class HoistSolenoid {
    public DoubleSolenoid solenoid;
    /** True menes open */
    public boolean isSolenoidForeword = false;

    public HoistSolenoid(int forwardChannel, int reverseChannel) {
      solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
    }

    public void changeBool() {
      if (isSolenoidForeword) {
        isSolenoidForeword = false;
      } else {
        isSolenoidForeword = true;
      }
    }
  }

  HoistSolenoid armSolenoid, hookSolenoid;

  private boolean isArmSolenoidForeword, isHookSolenoidForeword = false;

  public HoistSubsystem() {
    rightMotor = new VictorSP(PortMap.Hoist.RIGHT_MOTOR);
    leftMotor = new VictorSP(PortMap.Hoist.LEFT_MOTOR);

    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

    armSolenoid = new HoistSolenoid(PortMap.Hoist.ARM_SOLENOID_FOREWORD, PortMap.Hoist.ARM_SOLENOID_REVERSE);
    hookSolenoid = new HoistSolenoid(PortMap.Hoist.HOOK_SOLENOID_FOREWORD, PortMap.Hoist.HOOK_SOLENOID_REVERSE);

    configureMotors();
  }

  private void configureMotors() {
    rightMotor.setInverted(false);
    leftMotor.setInverted(false);

    hoistMotors = new MotorControllerGroup(rightMotor, leftMotor);
  }

  @Override
  public void periodic() {
  }

  public void setPercentageMotorOut(double out, double back) {
    hoistMotors.set(out - back);
  }

  public HoistSolenoid getHooksSolenoid() {
    return hookSolenoid;
  }

  public HoistSolenoid getArmSolenoid() {
    return armSolenoid;
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Hoist percentage out", hoistMotors.get());
    SmartDashboard.putBoolean("Hoist hook solenoid", getHooksSolenoid().isSolenoidForeword);
    SmartDashboard.putBoolean("Hoist arm solenoid", getArmSolenoid().isSolenoidForeword);
  }

}
