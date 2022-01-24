package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {

    private final static DriveTrainSubsystem INSTANCE = new DriveTrainSubsystem();

    private Joystick stick;

    private DifferentialDrive diffDrive;

    private CANSparkMax rightMotor, rightMotor1;
    private CANSparkMax leftMotor, leftMotor1;


    public static DriveTrainSubsystem getInstance() {
        return INSTANCE;
    }


    private DriveTrainSubsystem() {
        configureMotors();

        diffDrive = new DifferentialDrive(leftMotor, rightMotor);
    }

    private void configureMotors() {
        rightMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushed);
        rightMotor1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushed);
        rightMotor.restoreFactoryDefaults();
        rightMotor1.restoreFactoryDefaults();

        leftMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushed);
        leftMotor1 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushed);
        leftMotor1.restoreFactoryDefaults();
        leftMotor.restoreFactoryDefaults();

        leftMotor.setInverted(true);
        leftMotor1.setInverted(true);

        rightMotor1.follow(rightMotor);
        leftMotor1.follow(leftMotor);
    }
}

