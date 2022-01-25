package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class DriveTrainSubsystem extends SubsystemBase {

    private CANSparkMax rightMotor, rightMotorSlave;
    private CANSparkMax leftMasterMotor, leftMotorSlave;

    private Encoder leftEncoder, rightEncoder;

    public DriveTrainSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        rightMotor = new CANSparkMax(PortMap.DriveTrain.FRONT_RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushed);
        rightMotorSlave = new CANSparkMax(PortMap.DriveTrain.BACK_RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushed);
        rightMotor.restoreFactoryDefaults();
        rightMotorSlave.restoreFactoryDefaults();

        leftMasterMotor = new CANSparkMax(PortMap.DriveTrain.FRONT_LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushed);
        leftMotorSlave = new CANSparkMax(PortMap.DriveTrain.BACK_LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushed);
        leftMotorSlave.restoreFactoryDefaults();
        leftMasterMotor.restoreFactoryDefaults();

        leftMasterMotor.setInverted(true);
        leftMotorSlave.setInverted(true);

        rightMotorSlave.follow(rightMotor);
        leftMotorSlave.follow(leftMasterMotor);

        configureEncoders();
    }

    private void configureEncoders() {
        leftEncoder.setDistancePerPulse(
                Constants.DriveTrain.ROBOT_DISTANCE_PER_ROTATION / Constants.DriveTrain.ENCODER_TICK_RATE);
        leftEncoder.setMaxPeriod(Constants.DriveTrain.ENCODER_MIN_RATE);
        leftEncoder.setReverseDirection(Constants.DriveTrain.ENCODER_LEFT_REVERSE);
        leftEncoder.setSamplesToAverage(Constants.DriveTrain.ENCODER_SAMPLES_TO_AVERAGE);

        rightEncoder.setDistancePerPulse(
                Constants.DriveTrain.ROBOT_DISTANCE_PER_ROTATION / Constants.DriveTrain.ENCODER_TICK_RATE);
        rightEncoder.setMaxPeriod(Constants.DriveTrain.ENCODER_MIN_RATE);
        rightEncoder.setReverseDirection(Constants.DriveTrain.ENCODER_RIGHT_REVERSE);
        rightEncoder.setSamplesToAverage(Constants.DriveTrain.ENCODER_SAMPLES_TO_AVERAGE);
    }

    public void setLeftPercentOutput(double output) {
        leftMasterMotor.set(output);
    }

    public void setRightPercentOutput(double output) {
        rightMotor.set(output);
    }

    public void setMotorPercentageOutput(double leftOut, double rightOut, double turn) {
        setLeftPercentOutput(leftOut + turn);
        setRightPercentOutput(rightOut - turn);
    }

    /**
     * Function returns distance that left wheels of the robot has traveled.
     * 
     * @return The distance traveled in mm.
     */
    public double getLeftEncoderDistance() {
        return leftEncoder.getDistance();
    }

    /**
     * Function returns distance that left wheels of the robot has traveled.
     * 
     * @return The distance traveled in mm.
     */
    public double getRightEncoderDistance() {
        return rightEncoder.getDistance();
    }

    /**
     * Gets the current rate of the encoder
     * 
     * @return rate in ticks.
     */
    public double getLeftEncoderRate() {
        return leftEncoder.getRate();
    }

    /**
     * Gets the current rate of the encoder
     * 
     * @return rate in ticks.
     */
    public double getRightEncoderRate() {
        return rightEncoder.getRate();
    }

    /** Resets the encoders to read a distance of zero */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Left encoder rate", getLeftEncoderRate());
        SmartDashboard.putNumber("Right encoder rate", getRightEncoderRate());
        SmartDashboard.putNumber("Left encoder distance", getLeftEncoderDistance());
        SmartDashboard.putNumber("Right encoder distance", getRightEncoderDistance());
    }
}
