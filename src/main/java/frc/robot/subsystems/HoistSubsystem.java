// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.commands.hoist.ToggleSolenoid;

public class HoistSubsystem extends SubsystemBase {
  /** Creates a new Hoist. */
  public static class HoistSolenoid {
    public DoubleSolenoid solenoid;
    /** True menes open */
    public boolean isSolenoidForeword = true;

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

  private Compressor compressor;
  private HoistSolenoid armSolenoid, hookSolenoid;

  private VictorSP rightMotor, leftMotor;
  private MotorControllerGroup hoistMotors;

  private Encoder hoistEncoder;
  private DigitalInput limitSwitch;

  private PIDController PIDMotorController;

  public HoistSubsystem() {
    limitSwitch = new DigitalInput(6);

    configurePneumatics();
    configureMotors();
    configureEncoder();
  }

  private void configurePneumatics() {
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

    armSolenoid = new HoistSolenoid(PortMap.Hoist.ARM_SOLENOID_FOREWORD, PortMap.Hoist.ARM_SOLENOID_REVERSE);
    hookSolenoid = new HoistSolenoid(PortMap.Hoist.HOOK_SOLENOID_FOREWORD, PortMap.Hoist.HOOK_SOLENOID_REVERSE);

    PIDMotorController = new PIDController(
            Constants.Hoist.PID.KP,
            Constants.Hoist.PID.KI,
            Constants.Hoist.PID.KD
    );

    configureMotors();

  }

  private void configureMotors() {
    rightMotor = new VictorSP(PortMap.Hoist.RIGHT_MOTOR);
    leftMotor = new VictorSP(PortMap.Hoist.LEFT_MOTOR);

    leftMotor.setInverted(Constants.Hoist.IS_LEFT_MOTOR_INVERTED);
    rightMotor.setInverted(Constants.Hoist.IS_RIGHT_MOTOR_INVERTED);

    hoistMotors = new MotorControllerGroup(rightMotor, leftMotor);
  }

  private void configureEncoder() {
    hoistEncoder = new Encoder(PortMap.Hoist.ENCODER_A, PortMap.Hoist.ENCODER_B);
    hoistEncoder.setDistancePerPulse(
        Constants.Hoist.DISTANCE_PER_ROTATION / Constants.DriveTrain.ENCODER_TICK_RATE);
    hoistEncoder.setMaxPeriod(Constants.Hoist.ENCODER_MIN_RATE);
    hoistEncoder.setReverseDirection(Constants.Hoist.ENCODER_REVERSE);
    hoistEncoder.setSamplesToAverage(Constants.Hoist.ENCODER_SAMPLES_TO_AVERAGE);

    hoistEncoder.reset();
  }

  @Override
  public void periodic() {
    resetOnLimitSwitch();
  }

  private boolean limitSwitchState = false;

  public void resetOnLimitSwitch() {

    if (limitSwitch.get() && !limitSwitchState) {
      resetEncoder();
      limitSwitchState = true;
    }

    if (!limitSwitch.get()) {
      limitSwitchState = false;
    }

  }
  // limits pneumatics by how many tricks they can eject
  public void limitPneumatics(double joystickValue) {

    double limitOfTricks = 10 * Constants.Hoist.DISTANCE_PER_ROTATION; // limits by how many tricks pneumatics can eject
    double secLimitOftricks = 4 * Constants.Hoist.DISTANCE_PER_ROTATION;

    if (limitSwitchState = true) {
      if (joystickValue < 0.0 && hoistEncoder.getDistance() > limitOfTricks) {
        armSolenoid.solenoid.toggle();
      }
    } else {
      if (joystickValue < 0.0 && hoistEncoder.getDistance() > secLimitOftricks) {
        armSolenoid.solenoid.toggle();
      }
    }

  }

  public void setPercentageMotorOut(double out, double back) {
    out = out - back;

    // if (hoistEncoder.getRaw() < 0 && out < 0) {
    // out = 0;
    // } else if (hoistEncoder.getRaw() >= Constants.Hoist.MAX_ENCODER_VALUE && out
    // > 0) {
    // out = 0;
    // }

    hoistMotors.set(out);
  }

  public HoistSolenoid getHooksSolenoid() {
    return hookSolenoid;
  }

  public HoistSolenoid getArmSolenoid() {
    return armSolenoid;
  }

  public int getEncoderRaw() {
    return hoistEncoder.getRaw();
  }

  public void resetEncoder() {
    hoistEncoder.reset();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Hoist percentage out", hoistMotors.get());
    SmartDashboard.putBoolean("Hoist hook solenoid", getHooksSolenoid().isSolenoidForeword);
    SmartDashboard.putBoolean("Hoist arm solenoid", getArmSolenoid().isSolenoidForeword);
    SmartDashboard.putNumber("Hoist encoder", getEncoderRaw());
    SmartDashboard.putBoolean("limit switch", limitSwitch.get());
    SmartDashboard.putBoolean("limit switch state", limitSwitchState);
  }

  public void resetSubsystem() {
    CommandScheduler.getInstance().schedule(new ToggleSolenoid(this, getArmSolenoid()));
  }

}
