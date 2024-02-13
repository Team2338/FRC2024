package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class Shooter extends SubsystemBase {
    public static CANSparkMax shooter;
    public static SparkPIDController pidMainShooter;

    public static CANSparkMax shooterRotationController;
    public static PIDController pidRotation;
    public static CANcoder rotationEncoder;

    private double targetPosition;

    public Shooter() throws Exception {
        shooter = new CANSparkMax(RobotMap.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        configShooter();

        shooterRotationController = new CANSparkMax(RobotMap.SHOOTER_ANGLE_ID, CANSparkLowLevel.MotorType.kBrushless);

        rotationEncoder = new CANcoder(RobotMap.SHOOTER_ANGLE_ENCODER_ID);

        configShooterRotation();

        targetPosition = getPosition();
//        shooterAngle.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);

        pidShooterAngle = shooterAngle.getPIDController();
        pidShooterAngle.setP(Constants.ShooterRotation.kP);
        pidShooterAngle.setFF(Constants.ShooterRotation.FF);

        angleEncoder = new CANcoder(RobotMap.SHOOTER_ANGLE_ENCODER_ID);
        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(Constants.ShooterRotation.ENCODER_OFFSET_ABSOLUTE)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));

        // check constants are valid
        if (Constants.ShooterRotation.MIN_LIMIT_ABSOLUTE < Constants.ShooterRotation.HARD_STOP_ABSOLUTE){
            throw new Exception("Shooter MIN_LIMIT_ABSOLUTE needs to be greater than HARD_STOP_ABSOLUTE");
        }
    }

    public void setVoltage(double volt) {
//        shooter.setVoltage(volt);
        shooter.set(volt);
    }

    public double getVoltage() {
        return shooter.getBusVoltage();
    }

    public void setShooterRPM(double rpm) {
        pidMainShooter.setReference(rpm, CANSparkBase.ControlType.kVelocity);
    }

    public double getShooterRPM() {
        return shooter.getEncoder().getVelocity();
    }

    public String getShooterRPM_Shuffleboard() {
        return String.format("%12.0f", getShooterRPM());
    }

    /**
     * Rotates shooter <br>
     * positive (+) value is clockwise (shoots lower)<br>
     *
     * negative (-) value is counterclockwise (shoots higher)
     * @param percent  The percentage of motor power
     */
    public void setRotationPercentMove(double percent) {
        shooterRotationController.set(percent + Constants.Shooter.ROTATION_FF);
    }

    /**
     * Use PID to move the shooter angle to position
     */
    public void PIDRotationMove() {
        double pidOutput = pidRotation.calculate(rotationEncoder.getAbsolutePosition().getValueAsDouble(), targetPosition);
        shooterRotationController.set(pidOutput + Constants.Shooter.ROTATION_FF);
    }

    /**
     * Get the current position of the arm in ticks
     * @return arm position in ticks
     */
    public double getPosition(){
        return rotationEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * setZeroOffset <br>
     * Method first retrieves the current configuration<br>
     * and then updates the zero offset<br>
     *
     * @param offset  The value between -1 and 1 to offset the zero position of the encoder
     */
    public void setZeroOffset(double offset) {
        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs();
        angleEncoder.getConfigurator().refresh(magSensorConfig);
        magSensorConfig.withMagnetOffset(offset);
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));
    }

    public double degreesToAbsolute(double degrees){
        return (degrees - Constants.ShooterRotation.MIN_LIMIT_DEGREES) * Constants.ShooterRotation.ABSOLUTE_PER_DEGREE + Constants.ShooterRotation.MIN_LIMIT_ABSOLUTE;
    }
    public double absoluteToDegrees(double absolute){
        return ( (absolute - Constants.ShooterRotation.MIN_LIMIT_ABSOLUTE)/ Constants.ShooterRotation.ABSOLUTE_PER_DEGREE +  Constants.ShooterRotation.MIN_LIMIT_DEGREES);
    }

    /**
     * Set the target position of the arm in ticks
     * @param pos target position in ticks
     */
    public void setTargetPosition(double pos) {
        targetPosition = pos;
    }

    /**
     * Get the target position of the arm in ticks
     * @return target position in ticks
     */
    public double getTargetPosition() {return targetPosition;}

    /**
     * Get the current error of the arm PID
     * @return the current error of the arm PID
     */
    public double getPIDRotationError() {
        return getPosition() - targetPosition;
    }


    /**
     *  All the config setting for both ShooterAngle and MainShooter
     */
    public void configShooter() {
        shooter.restoreFactoryDefaults();
        shooter.setInverted(true);
        shooter.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pidMainShooter = shooter.getPIDController();

        pidMainShooter.setP(Constants.Shooter.kP);
        pidMainShooter.setFF(Constants.Shooter.FF);
        pidMainShooter.setI(Constants.Shooter.kI);
        pidMainShooter.setOutputRange(0,1);
    }

    public void configShooterRotation() {
        shooterRotationController.restoreFactoryDefaults();
        shooterRotationController.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shooterRotationController.setOpenLoopRampRate(.25);

        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        magSensorConfig.withMagnetOffset(-.133);
        magSensorConfig.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        rotationEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));

        pidRotation = new PIDController(Constants.Shooter.ROTATION_kP, Constants.Shooter.ROTATION_kI, Constants.Shooter.ROTATION_kD);
    }
}
