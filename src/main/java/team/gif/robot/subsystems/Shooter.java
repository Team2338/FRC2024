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
import team.gif.robot.RobotMap;

public class Shooter extends SubsystemBase {
    public static CANSparkMax shooter;
    public static SparkPIDController pidShooter;

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

        // check constants are valid
        if (Constants.ShooterRotation.MIN_LIMIT_ABSOLUTE < Constants.ShooterRotation.HARD_STOP_ABSOLUTE){
            throw new Exception("Shooter MIN_LIMIT_ABSOLUTE needs to be greater than HARD_STOP_ABSOLUTE");
        }

        // set the initial shooter rotation position so PID can hold the current position when first enabled
        targetPosition = getPosition();
    }

    public void setVoltage(double volt) {
//        shooter.setVoltage(volt);
        shooter.set(volt);
    }

    public double getVoltage() {
        return shooter.getBusVoltage();
    }

    public void setShooterRPM(double rpm) {
        pidShooter.setReference(rpm, CANSparkBase.ControlType.kVelocity);
    }

    public double getShooterRPM() {
        return shooter.getEncoder().getVelocity();
    }

    public String getShooterRPM_Shuffleboard() {
        return String.format("%12.0f", getShooterRPM());
    }

    public String getRotationDegrees_Shuffleboard() {
        return String.format("%12.1f", absoluteToDegrees(getPosition()));
    }

    /**
     * Rotates shooter <br>
     *
     * positive (+) value is clockwise (shoots lower)<br>
     * negative (-) value is counterclockwise (shoots higher)
     *
     * @param percent  The percentage of motor power
     */
    public void moveRotationPercentPower(double percent) {
        shooterRotationController.set(percent);
    }

    /**
     * Holds thr shooter rotation using FF <br
     */
    public void holdRotation() {
        shooterRotationController.set(Constants.ShooterRotation.FF);
    }

    /**
     * Use PID to move the shooter angle to absolute position between 0 and 1
     */
    public void PIDRotationMove() {
        double pidOutput = pidRotation.calculate(rotationEncoder.getAbsolutePosition().getValueAsDouble(), targetPosition);
        shooterRotationController.set(pidOutput + Constants.ShooterRotation.FF);
    }

    /**
     * Get the current position of the arm in absolute between 0 and 1
     * @return arm position in ticks
     */
    public double getPosition(){
        return rotationEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public String getPosition_Shuffleboard() {
        return String.format("%12.3f", getPosition());
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
        rotationEncoder.getConfigurator().refresh(magSensorConfig);
        magSensorConfig.withMagnetOffset(offset);
        rotationEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));
    }

    public double degreesToAbsolute(double degrees){
        return (degrees - Constants.ShooterRotation.MIN_LIMIT_DEGREES) * Constants.ShooterRotation.ABSOLUTE_PER_DEGREE + Constants.ShooterRotation.MIN_LIMIT_ABSOLUTE;
    }
    public double absoluteToDegrees(double absolute){
        return ( (absolute - Constants.ShooterRotation.MIN_LIMIT_ABSOLUTE)/ Constants.ShooterRotation.ABSOLUTE_PER_DEGREE +  Constants.ShooterRotation.MIN_LIMIT_DEGREES);
    }

    /**
     * Set the target position of the arm in absolute from 0 to 1
     * @param pos target position in ticks
     */
    public void setTargetPosition(double pos) {
        targetPosition = pos;
    }


    /**
     * Moves shooter rotation to the Far setpoint defined in constants.java
     */
    public void setRotationFar() {
        targetPosition = Constants.ShooterRotation.SETPOINT_FAR;
    }

    /**
     * Moves shooter rotation to the Mid setpoint defined in constants.java
     */
    public void setRotationMid() {
        targetPosition = Constants.ShooterRotation.SETPOINT_MID;
    }

    /**
     * Moves shooter rotation to the Near setpoint defined in constants.java
     */
    public void setRotationNear() {
        targetPosition = Constants.ShooterRotation.SETPOINT_NEAR;
    }

    /**
     * Moves shooter rotation to the Wall setpoint defined in constants.java
     */
    public void setRotationWall() {
        targetPosition = Constants.ShooterRotation.SETPOINT_WALL;
    }




    /**
     * Get the target position of the arm in absolute from 0 to 1
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
     *  All the config setting for Shooter (controller, pid)
     */
    public void configShooter() {
        shooter.restoreFactoryDefaults();
        shooter.setInverted(true);
        shooter.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pidShooter = shooter.getPIDController();

        pidShooter.setP(Constants.Shooter.kP);
        pidShooter.setFF(Constants.Shooter.FF);
        pidShooter.setI(Constants.Shooter.kI);
        pidShooter.setOutputRange(0,1);
    }

    /**
     *  All the config setting for Rotation (controller, sensor, rotation pid)
     */
    public void configShooterRotation() {
        shooterRotationController.restoreFactoryDefaults();
        shooterRotationController.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shooterRotationController.setOpenLoopRampRate(.25);

        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(Constants.ShooterRotation.ENCODER_OFFSET_ABSOLUTE)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        rotationEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));

        pidRotation = new PIDController(Constants.ShooterRotation.kP, Constants.ShooterRotation.kI, Constants.ShooterRotation.kD);
    }
}
