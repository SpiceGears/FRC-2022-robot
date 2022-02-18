package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class DriveTrainSubsystem extends SubsystemBase {

    private Spark rightMasterMotor, rightMotorSlave;
    private Spark leftMasterMotor, leftMotorSlave;

    private Encoder leftEncoder, rightEncoder;
    private ADXRS450_Gyro gyro;

    // The motors on the left side of the drive.
    private MotorControllerGroup leftMotors;

    // The motors on the right side of the drive.
    private MotorControllerGroup rightMotors;

    private DifferentialDrive diffDrive;
    private PIDController lPIDMotorController;
    private PIDController rPIDMotorController;

    private SimpleMotorFeedforward feedforward;

    // Odometry class for tracking robot pose
    private DifferentialDriveOdometry m_odometry;

    public DriveTrainSubsystem() {
        gyro = new ADXRS450_Gyro(Port.kMXP);

        gyro.calibrate();

        lPIDMotorController = new PIDController(
                Constants.DriveTrain.PID.LEFT_KP,
                Constants.DriveTrain.PID.LEFT_KI,
                Constants.DriveTrain.PID.LEFT_KD);
        rPIDMotorController = new PIDController(
                Constants.DriveTrain.PID.RIGHT_KP,
                Constants.DriveTrain.PID.RIGHT_KI,
                Constants.DriveTrain.PID.RIGHT_KD);

        feedforward = new SimpleMotorFeedforward(
                Constants.DriveTrain.Feedforward.KS,
                Constants.DriveTrain.Feedforward.KV,
                Constants.DriveTrain.Feedforward.KA);

        m_odometry = new DifferentialDriveOdometry(getHeading());

        configureMotors();
    }

    private void configureMotors() {
        rightMasterMotor = new Spark(PortMap.DriveTrain.FRONT_RIGHT_MOTOR);
        rightMotorSlave = new Spark(PortMap.DriveTrain.BACK_RIGHT_MOTOR);

        leftMasterMotor = new Spark(PortMap.DriveTrain.FRONT_LEFT_MOTOR);
        leftMotorSlave = new Spark(PortMap.DriveTrain.BACK_LEFT_MOTOR);

        leftMasterMotor.setInverted(Constants.DriveTrain.IS_LEFT_MASTER_INVERTED);
        leftMotorSlave.setInverted(Constants.DriveTrain.IS_LEFT_SLAVE_INVERTED);

        rightMasterMotor.setInverted(Constants.DriveTrain.IS_RIGHT_MASTER_INVERTED);
        rightMotorSlave.setInverted(Constants.DriveTrain.IS_RIGHT_SLAVE_INVERTED);

        leftMotors = new MotorControllerGroup(leftMasterMotor, leftMotorSlave);
        rightMotors = new MotorControllerGroup(rightMasterMotor, rightMotorSlave);

        diffDrive = new DifferentialDrive(leftMotors, rightMotors);

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

        resetEncoders();
    }

    public void setRawMotorPercentageOutput(double leftOut, double rightOut, double turn) {
        if (Math.abs(leftOut) < Constants.DriveTrain.MOTOR_MIN_PERCENTAGE_OUT)
            leftOut = 0;
        if (Math.abs(rightOut) < Constants.DriveTrain.MOTOR_MIN_PERCENTAGE_OUT)
            rightOut = 0;
        if (Math.abs(turn) < Constants.DriveTrain.MOTOR_MIN_PERCENTAGE_OUT)
            turn = 0;
        leftMotors.set(leftOut - turn);
        rightMotors.set(rightOut + turn);
    }

    /**
     * Function converts percentage outputs to voltage output.
     * 
     * @param speedInput 1 = 100% forward
     * @param turnInput  1 = 100% right turn
     */
    public void joystickDriveTrain(double speedInput, double turnInput) {
        double lOutput = 0;
        double rOutput = 0;

        if (Math.abs(speedInput) < Constants.DriveTrain.MOTOR_MIN_PERCENTAGE_OUT)
            speedInput = 0;
        if (Math.abs(turnInput) < Constants.DriveTrain.MOTOR_MIN_PERCENTAGE_OUT)
            turnInput = 0;

        SmartDashboard.putNumber("In speedInput", speedInput);
        SmartDashboard.putNumber("In turnInput", turnInput);

        // // no turn
        // if (turnInput == 0) {
        // lOutput = speedInput;
        // rOutput = speedInput;
        // } else {

        // if (Math.abs(speedInput) <= 0.4) {
        // lOutput = speedInput - turnInput;
        // rOutput = speedInput + turnInput;
        // }
        // // Turn and speed added are less or equal than 100%
        // else if (Math.abs(speedInput) + Math.abs(turnInput) <= 1
        // && Math.abs(speedInput) + Math.abs(turnInput) != 0) {
        // // Ride foreword
        // if (speedInput > 0) {
        // // turning right
        // if (turnInput > 0) {
        // lOutput = speedInput + turnInput;
        // rOutput = speedInput;
        // }
        // // turning left
        // else if (turnInput < 0) {
        // lOutput = speedInput;
        // rOutput = speedInput - turnInput;
        // }
        // } else {
        // // turning right
        // if (turnInput > 0) {
        // lOutput = speedInput - turnInput;
        // rOutput = speedInput;
        // }
        // // turning left
        // else if (turnInput < 0) {
        // lOutput = speedInput;
        // rOutput = speedInput + turnInput;
        // }
        // }
        // }
        // // Turn and speed added are greater than 100%
        // else if (Math.abs(speedInput) + Math.abs(turnInput) > 1
        // && Math.abs(speedInput) + Math.abs(turnInput) != 0) {
        // if (Math.abs(speedInput) >= 1) {
        // // Ride foreword
        // if (speedInput > 0) {
        // // turning right
        // if (turnInput > 0) {
        // lOutput = speedInput;
        // rOutput = speedInput - turnInput;
        // }
        // // turning left
        // else if (turnInput < 0) {
        // lOutput = speedInput + turnInput;
        // rOutput = speedInput;
        // }
        // }
        // // Ride backward
        // else {
        // // turning right
        // if (turnInput > 0) {
        // lOutput = speedInput;
        // rOutput = speedInput + turnInput;
        // }
        // // turning left
        // else if (turnInput < 0) {
        // lOutput = speedInput - turnInput;
        // rOutput = speedInput;
        // }
        // }
        // } else {
        // // Ride foreword
        // if (speedInput > 0) {
        // // turning right
        // if (turnInput > 0) {
        // lOutput = 1;
        // rOutput = speedInput - (speedInput + turnInput - 1);
        // }
        // // turning left
        // else if (turnInput < 0) {
        // lOutput = speedInput - (speedInput - turnInput - 1);
        // rOutput = 1;
        // }
        // }
        // // Ride backward
        // else {
        // // turning right
        // if (turnInput > 0) {
        // lOutput = -1;
        // rOutput = -speedInput + (speedInput + turnInput - 1);
        // }
        // // turning left
        // else if (turnInput < 0) {
        // lOutput = -speedInput + (speedInput - turnInput - 1);
        // rOutput = -1;
        // }
        // }
        // }
        // }
        // }

        lOutput = speedInput + turnInput;
        rOutput = speedInput - turnInput;

        lOutput *= Constants.DriveTrain.MAX_ROBOT_SPEED;
        rOutput *= Constants.DriveTrain.MAX_ROBOT_SPEED;

        SmartDashboard.putNumber("Out left", lOutput);
        SmartDashboard.putNumber("Out right", rOutput);

        driveFeedforwardPID(lOutput, rOutput);
    }

    /**
     * Function uses Feedforward and PID controllers to tank drive robot
     * 
     * @param leftSetpoint  voltage setpoint m/s
     * @param rightSetpoint voltage setpoint m/s
     */
    public void driveFeedforwardPID(double leftSetpoint, double rightSetpoint) {
        // double leftOut = feedforward.calculate(leftSetpoint)
        // + lPIDMotorController.calculate(getLeftEncoderRate(), leftSetpoint / 100);
        // double rightOut = feedforward.calculate(rightSetpoint)
        // + rPIDMotorController.calculate(getRightEncoderRate(), rightSetpoint / 100);
        double leftOut = lPIDMotorController.calculate(getLeftEncoderRate(),
                leftSetpoint * 1000);
        double rightOut = rPIDMotorController.calculate(getRightEncoderRate(),
                rightSetpoint * 1000);

        tankDriveVolts(leftOut * 12, rightOut * 12);

        SmartDashboard.putNumber("getLeftEncoderRate()", -getLeftEncoderRate());
        SmartDashboard.putNumber("getRightEncoderRate()", getRightEncoderRate());

        SmartDashboard.putNumber("leftSetpoint * 1000", -leftSetpoint * 1000);
        SmartDashboard.putNumber("leftSetpoint * 1000", leftSetpoint * 1000);

        SmartDashboard.putNumber("Left driveFeedforwardPID", leftOut);
        SmartDashboard.putNumber("Right driveFeedforwardPID", rightOut);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        diffDrive.feed();
    }

    /**
     * Function returns distance that left wheels of the robot has traveled.
     * 
     * @return The distance traveled in mm.
     */
    public double getLeftEncoderDistance() {
        return -leftEncoder.getDistance();
    }

    /**
     * Function returns distance that left wheels of the robot has traveled.
     * 
     * @return The distance traveled in mm.
     */
    public double getRightEncoderDistance() {
        return -rightEncoder.getDistance();
    }

    /**
     * Gets the current rate of the encoder [mm/s]
     * 
     * @return rate in ticks.
     */
    public double getLeftEncoderRate() {
        return -leftEncoder.getRate();
    }

    /**
     * Gets the current rate of the encoder
     * 
     * @return rate in ticks.
     */
    public double getRightEncoderRate() {
        return -rightEncoder.getRate();
    }

    /**
     * Gets the current speed of the encoder km/h
     * 
     * @return rate in ticks.
     */
    public double getRightEncoderSpeed() {
        return -rightEncoder.getRate() * 0.0036;
    }

    /**
     * Gets the current speed of the encoder km/h
     * 
     * @return rate in ticks.
     */
    public double getLeftEncoderSpeed() {
        return -leftEncoder.getRate() * 0.0036;
    }

    /** Resets the encoders to read a distance of zero */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getGyroAngle() {
        return gyro.getAngle();
    }

    // public Rotation2d getGyroRotation2D() {
    // return gyro.getRotation2d();
    // }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, getHeading());
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-getGyroAngle());
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    public void updateOdometry() {
        m_odometry.update(
                getHeading(), getLeftEncoderDistance() * 0.001, getRightEncoderDistance() * 0.001);
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void calibrateGyro() {
        gyro.calibrate();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate();
    }

    public void stopMotors() {
        tankDriveVolts(0, 0);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Left encoder rate", getLeftEncoderRate());
        SmartDashboard.putNumber("Right encoder rate", getRightEncoderRate());
        SmartDashboard.putNumber("Left encoder distance", getLeftEncoderDistance());
        SmartDashboard.putNumber("Right encoder distance", getRightEncoderDistance());
        SmartDashboard.putNumber("Left encoder speed", getLeftEncoderSpeed());
        SmartDashboard.putNumber("Right encoder speed", getRightEncoderSpeed());
        SmartDashboard.putNumber("Gyro angle", getGyroAngle());
        SmartDashboard.putNumber("Gyro rate", gyro.getRate());
        SmartDashboard.putBoolean("Gyro", gyro.isConnected());
        SmartDashboard.putNumber("Gyro getHeading", getHeading().getDegrees());
        SmartDashboard.putNumber("Gyro getHeading", getHeading().getRadians());
    }

    public void resetSubsystem() {
        stopMotors();
        resetEncoders();
        resetGyro();
    }
}
