package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class DriveTrainSubsystem extends SubsystemBase {

    private Spark rightMotor, rightMotorSlave;
    private Spark leftMasterMotor, leftMotorSlave;

    private Encoder leftEncoder, rightEncoder;

    public DriveTrainSubsystem() {
        // configureMotors();
    }

    private void configureMotors() {
        rightMotor = new Spark(PortMap.DriveTrain.FRONT_RIGHT_MOTOR);
        rightMotorSlave = new Spark(PortMap.DriveTrain.BACK_RIGHT_MOTOR);

        leftMasterMotor = new Spark(PortMap.DriveTrain.FRONT_LEFT_MOTOR);
        leftMotorSlave = new Spark(PortMap.DriveTrain.BACK_LEFT_MOTOR);

        leftMasterMotor.setInverted(true);
        leftMotorSlave.setInverted(true);

        // configureEncoders();
    }

    private void configureEncoders() {
        leftEncoder = new Encoder(PortMap.DriveTrain.LEFT_ENCODER_A, PortMap.DriveTrain.LEFT_ENCODER_B);
        leftEncoder.setDistancePerPulse(
                Constants.DriveTrain.ROBOT_DISTANCE_PER_ROTATION / Constants.DriveTrain.ENCODER_TICK_RATE);
        leftEncoder.setMaxPeriod(Constants.DriveTrain.ENCODER_MIN_RATE);
        leftEncoder.setReverseDirection(Constants.DriveTrain.ENCODER_LEFT_REVERSE);
        leftEncoder.setSamplesToAverage(Constants.DriveTrain.ENCODER_SAMPLES_TO_AVERAGE);

        rightEncoder = new Encoder(PortMap.DriveTrain.RIGHT_ENCODER_A, PortMap.DriveTrain.RIGHT_ENCODER_B);
        rightEncoder.setDistancePerPulse(
                Constants.DriveTrain.ROBOT_DISTANCE_PER_ROTATION / Constants.DriveTrain.ENCODER_TICK_RATE);
        rightEncoder.setMaxPeriod(Constants.DriveTrain.ENCODER_MIN_RATE);
        rightEncoder.setReverseDirection(Constants.DriveTrain.ENCODER_RIGHT_REVERSE);
        rightEncoder.setSamplesToAverage(Constants.DriveTrain.ENCODER_SAMPLES_TO_AVERAGE);
    }

    public void setLeftPercentOutput(double output) {
        leftMasterMotor.set(output);
        leftMotorSlave.set(output);

        SmartDashboard.putNumber("Left motor out", output);
    }

    public void setRightPercentOutput(double output) {
        rightMotor.set(output);
        rightMotorSlave.set(output);

        SmartDashboard.putNumber("Right motor out", output);
    }

    public void setMotorPercentageOutput(double leftOut, double rightOut, double turn) {
        if (Math.abs(leftOut) < Constants.DriveTrain.MOTOR_MIN_OUT) {
            leftOut = 0;
        }
        if (Math.abs(rightOut) < Constants.DriveTrain.MOTOR_MIN_OUT) {
            rightOut = 0;
        }
        if (Math.abs(turn) < Constants.DriveTrain.MOTOR_MIN_OUT) {
            turn = 0;
        }
        setLeftPercentOutput(leftOut - turn);
        setRightPercentOutput(rightOut + turn);
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
        // SmartDashboard.putNumber("Left encoder rate", getLeftEncoderRate());
        // SmartDashboard.putNumber("Right encoder rate", getRightEncoderRate());
        // SmartDashboard.putNumber("Left encoder distance", getLeftEncoderDistance());
        // SmartDashboard.putNumber("Right encoder distance",
        // getRightEncoderDistance());
    }
}
