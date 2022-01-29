package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class IntakeSubsystem extends SubsystemBase {
    
    private VictorSP intakeMotor;
    private Encoder intakeEncoder;
    private DoubleSolenoid intakeSolenoid;

    public IntakeSubsystem() {
        configureMotors();
        configureEncoders();
        configureSolenoids();
    }

    private void configureMotors() {

        intakeMotor = new VictorSP          (PortMap.Intake.INTAKE_MOTOR);
    }

    private void configureEncoders() { 

        // NOT CURRENT ARGUMENTS
        intakeEncoder = new Encoder         (PortMap.Intake.ENCODER_A, PortMap.Intake.ENCODER_B);
        intakeEncoder.setDistancePerPulse   (Constants.Intake.ROBOT_DISTANCE_PER_ROTATION / Constants.Intake.ENCODER_TICK_RATE);
        intakeEncoder.setMaxPeriod          (Constants.Intake.ENCODER_MIN_RATE);
        intakeEncoder.setReverseDirection   (Constants.Intake.ENCODER_REVERSE);
        intakeEncoder.setSamplesToAverage   (Constants.Intake.ENCODER_SAMPLES_TO_AVERAGE);
    }
    
    private void configureSolenoids() {

        intakeSolenoid = new DoubleSolenoid (0, null, 0, 0); // NOT CURRENT ARGUMENTS
        intakeSolenoid.set                  (Value.kOff);
    }

    // Sets the output of intake motor
    private void setIntakeMotorOutput(double output) {
        intakeMotor.set(output);
    }

    // Resets the intake encoder
    private void resetEncoders() {
        intakeEncoder.reset();
    }

    // Extends the intake solenoid
    private void extendIntakeSolenoid() {
        intakeSolenoid.set(Value.kForward);
    }

    // Retracts the intake solenoid
    private void retractIntakeSolenoid() {
        intakeSolenoid.set(Value.kReverse);
    }

    public Value getIntakeSolenoidState() {
        return intakeSolenoid.get();
    }

    public void updateSmartDashboard() {
    }
}
