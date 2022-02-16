package frc.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
//  import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PortMap;

public class IntakeSubsystem extends SubsystemBase {

    private VictorSP intakeMotor;
    // private Encoder intakeEncoder;
    private DoubleSolenoid intakeSolenoid;

    public boolean isIntakeOpen = false;

    public IntakeSubsystem() {
        configureMotors();
        // configureEncoders();
        configureSolenoids();
    }

    private void configureMotors() {
        intakeMotor = new VictorSP(PortMap.Intake.INTAKE_MOTOR);
    }

    /*
     * private void configureEncoders() {
     * 
     * // NOT CURRENT
     * intakeEncoder = new Encoder (PortMap.Intake.ENCODER_A,
     * PortMap.Intake.ENCODER_B);
     * intakeEncoder.setDistancePerPulse
     * (Constants.Intake.ROBOT_DISTANCE_PER_ROTATION /
     * Constants.Intake.ENCODER_TICK_RATE);
     * intakeEncoder.setMaxPeriod (Constants.Intake.ENCODER_MIN_RATE);
     * intakeEncoder.setReverseDirection (Constants.Intake.ENCODER_REVERSE);
     * intakeEncoder.setSamplesToAverage
     * (Constants.Intake.ENCODER_SAMPLES_TO_AVERAGE);
     * }
     */

    private void configureSolenoids() {
        intakeSolenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 1, 0); // NOT CURRENT
        intakeSolenoid.set(Value.kOff);
    }

    TimerTask turnSolenoidOff = new TimerTask() {
        public void run() {
            intakeSolenoid.set(Value.kOff);
        }
    };

    // Sets the output of intake motor
    public void setIntakeOutput(double speed) {
        if (isIntakeOpen) {
            intakeMotor.set(speed);
        } else {
            intakeMotor.set(0);
        }
    }

    /*
     * // Resets the intake encoder
     * public void resetEncoders() {
     * intakeEncoder.reset();
     * }
     */

    // Opens the intake
    public void openIntake() {
        intakeSolenoid.set(Value.kForward);
        isIntakeOpen = true;
    }

    // Closes the intake and turns off intake motor
    public void closeIntake() {
        intakeMotor.set(0);
        intakeSolenoid.set(Value.kReverse);
        isIntakeOpen = false;
    }

    public void toggleIntake() {
        if (isIntakeOpen) {
            closeIntake();
        } else if (!isIntakeOpen) {
            openIntake();
        }
        new Timer().schedule(
                new TimerTask() {
                    public void run() {
                        intakeSolenoid.set(Value.kOff);
                    }
                }, Constants.Intake.INTAKE_SOLENOID_DELAY);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("intakeoutput", intakeMotor.get());
    }
}
