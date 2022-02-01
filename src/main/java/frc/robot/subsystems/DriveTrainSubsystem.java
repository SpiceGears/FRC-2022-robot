package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class DriveTrainSubsystem extends SubsystemBase {

    private Spark rightMasterMotor, rightMotorSlave;
    private Spark leftMasterMotor, leftMotorSlave;

    private Encoder leftEncoder, rightEncoder;

    private PIDController lPIDMotorController = new PIDController(
            Constants.DriveTrain.PID.LEFT_KP,
            Constants.DriveTrain.PID.LEFT_KI,
            Constants.DriveTrain.PID.LEFT_KD);
    private PIDController rPIDMotorController = new PIDController(
            Constants.DriveTrain.PID.RIGHT_KP,
            Constants.DriveTrain.PID.RIGHT_KI,
            Constants.DriveTrain.PID.RIGHT_KD);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.DriveTrain.Feedforward.KS,
            Constants.DriveTrain.Feedforward.KV,
            Constants.DriveTrain.Feedforward.KA);

    public DriveTrainSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        rightMasterMotor = new Spark(PortMap.DriveTrain.FRONT_RIGHT_MOTOR);
        rightMotorSlave = new Spark(PortMap.DriveTrain.BACK_RIGHT_MOTOR);

        leftMasterMotor = new Spark(PortMap.DriveTrain.FRONT_LEFT_MOTOR);
        leftMotorSlave = new Spark(PortMap.DriveTrain.BACK_LEFT_MOTOR);

        leftMasterMotor.setInverted(true);
        leftMotorSlave.setInverted(true);

        configureEncoders();
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
        rightMasterMotor.set(output);
        rightMotorSlave.set(output);

        SmartDashboard.putNumber("Right motor out", output);
    }

    public void setRawMotorPercentageOutput(double leftOut, double rightOut, double turn) {
        if (Math.abs(leftOut) < Constants.DriveTrain.MOTOR_MIN_PERCENTAGE_OUT)
            leftOut = 0;
        if (Math.abs(rightOut) < Constants.DriveTrain.MOTOR_MIN_PERCENTAGE_OUT)
            rightOut = 0;
        if (Math.abs(turn) < Constants.DriveTrain.MOTOR_MIN_PERCENTAGE_OUT)
            turn = 0;
        setLeftPercentOutput(leftOut - turn);
        setRightPercentOutput(rightOut + turn);
    }

    /**
     * Function converts percentage outputs to voltage output.
     * 
     * @param speedInput 1 = 100% forward
     * @param turnInput  1 = 100% right turn
     */
    public void joystickDriveTrain(double speedInput, double turnInput) {
        double lOutput, rOutput;

    }

    /**
     * Function uses Feedforward and PID controllers to tank drive robot
     * 
     * @param leftSetpoint  voltage setpoint m/s
     * @param rightSetpoint voltage setpoint m/s
     */
    public void driveFeedforwardPID(double leftSetpoint, double rightSetpoint) {
        double leftOut = feedforward.calculate(leftSetpoint)
                + lPIDMotorController.calculate(getLeftEncoderRate() / 100, leftSetpoint);
        double rightOut = feedforward.calculate(rightSetpoint)
                + rPIDMotorController.calculate(getRightEncoderRate() / 100, rightSetpoint);

        if (Math.abs(leftOut) < Constants.DriveTrain.MOTOR_MIN_VOLTAGE_OUT)
            leftOut = 0;
        if (Math.abs(rightOut) < Constants.DriveTrain.MOTOR_MIN_VOLTAGE_OUT)
            rightOut = 0;
        leftMasterMotor.setVoltage(leftOut);
        rightMasterMotor.setVoltage(rightOut);

        SmartDashboard.putNumber("Left driveFeedforwardPID", leftOut);
        SmartDashboard.putNumber("Right driveFeedforwardPID", rightOut);
    }

    /**
     * Function returns distance that left wheels of the robot h aas traveled.
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
     * Gets the current rate of the encoder [mm/s]
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
