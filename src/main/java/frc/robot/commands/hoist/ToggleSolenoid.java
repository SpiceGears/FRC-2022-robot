// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hoist;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HoistSubsystem;
import frc.robot.subsystems.HoistSubsystem.HoistSolenoid;

public class ToggleSolenoid extends CommandBase {

  private HoistSolenoid hoistSolenoid;

  /** Creates a new ToogleHook. */
  public ToggleSolenoid(HoistSubsystem hoistSubsystem, HoistSolenoid hoistSolenoid) {
    this.hoistSolenoid = hoistSolenoid;
    addRequirements(hoistSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (hoistSolenoid.isSolenoidForeword) {
      putSolenoidReverse();
    } else {
      putSolenoidForeword();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  private void putSolenoidReverse() {
    hoistSolenoid.changeBool();
    hoistSolenoid.solenoid.set(Value.kReverse);
    putSolenoidOff(hoistSolenoid.solenoid);
  }

  private void putSolenoidForeword() {
    hoistSolenoid.changeBool();
    hoistSolenoid.solenoid.set(Value.kForward);
    putSolenoidOff(hoistSolenoid.solenoid);
  }

  private void putSolenoidOff(DoubleSolenoid doubleSolenoid) {
    new Timer().schedule(
        new TimerTask() {
          public void run() {
            doubleSolenoid.set(Value.kOff);
          }
        }, Constants.Hoist.SOLENOID_SWITCH_OFF_DELAY);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hoistSolenoid.solenoid.get() == Value.kOff;
  }
}
