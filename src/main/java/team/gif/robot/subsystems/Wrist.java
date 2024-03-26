package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.lib.shootParams;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class Wrist extends SubsystemBase {
    public static CANSparkMax motor;
    public static PIDController pidController;
    public static CANcoder wristEncoder; // The wrist uses an external CANcoder for its encoder

    private double targetPosition; // Rotation

    public Wrist() throws Exception {
        motor = new CANSparkMax(RobotMap.WRIST_ID, CANSparkLowLevel.MotorType.kBrushless);

        wristEncoder = new CANcoder(RobotMap.WRIST_ENCODER_ID);
        configWrist();

        // check constants are valid
        if (Constants.Wrist.MIN_LIMIT_ABSOLUTE < Constants.Wrist.HARD_STOP_ABSOLUTE){
            throw new Exception("Shooter MIN_LIMIT_ABSOLUTE needs to be greater than HARD_STOP_ABSOLUTE");
        }

        // set the initial shooter rotation position so PID can hold the current position when first enabled
        targetPosition = getPosition();
    }

    public String getWristDegrees_Shuffleboard() {
        return String.format("%4.1f", absoluteToDegrees(getPosition()));
    }

    /**
     * Rotates shooter <br>
     *
     * positive (+) value is clockwise (shoots lower)<br>
     * negative (-) value is counterclockwise (shoots higher)
     *
     * @param percent  The percentage of motor power
     */
    public void moveWristPercentPower(double percent) {
        if(getPosition() < Constants.Wrist.KILL_LIMIT_ABSOLUTE) {
            motor.set(percent);
        } else {
            motor.set(0);
        }
    }

    /**
     * Holds thr shooter rotation using FF <br
     */
    public void holdWrist() {
        motor.set(Constants.Wrist.FF);
    }

    /**
     * Use PID to move the shooter angle to absolute position between 0 and 1
     */
    public void PIDWristMove() {
        double pidOutput = pidController.calculate(wristEncoder.getAbsolutePosition().getValueAsDouble(), targetPosition);
        motor.set(pidOutput + Constants.Wrist.FF);
    }

    /**
     * Get the current position of the arm in absolute between 0 and 1
     * @return arm position in ticks
     */
    public double getPosition(){
        return wristEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public String getPosition_Shuffleboard() {
        return String.format("%3.3f", getPosition());
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
        wristEncoder.getConfigurator().refresh(magSensorConfig);
        magSensorConfig.withMagnetOffset(offset);
        wristEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));
    }

    public double degreesToAbsolute(double degrees){
        return (degrees - Constants.Wrist.MIN_LIMIT_DEGREES) * Constants.Wrist.ABSOLUTE_PER_DEGREE + Constants.Wrist.MIN_LIMIT_ABSOLUTE;
    }
    
    public double absoluteToDegrees(double absolute){
        return ( (absolute - Constants.Wrist.MIN_LIMIT_ABSOLUTE)/ Constants.Wrist.ABSOLUTE_PER_DEGREE +  Constants.Wrist.MIN_LIMIT_DEGREES);
    }

    /**
     * Set the target position of the arm in absolute from 0 to 1
     * @param pos target position in ticks
     */
    public void setTargetPosition(double pos) {
        targetPosition = pos;
    }

    /**
     * Moves shooter rotation to the Far3 setpoint defined in constants.java
     */
    public void setWristFar3Position() {
        Robot.nextShot = shootParams.FAR3;
    }

    /**
     * Moves shooter rotation to the Far2 setpoint defined in constants.java
     */
    public void setWristFar2Position() {
        Robot.nextShot = shootParams.FAR2;
    }

    /**
     * Moves shooter rotation to the MidMidFar setpoint defined in constants.java
     */
    public void setWristMidMidFarPosition() {
        Robot.nextShot = shootParams.MIDMIDFAR;
    }

    /**
     * Moves shooter rotation to the Far setpoint defined in constants.java
     */
    public void setWristFarPosition() {
        Robot.nextShot = shootParams.FAR;
    }

    /**
     * Moves shooter rotation to the Near setpoint defined in constants.java
     */
    public void setWristMidFarPosition() {
        Robot.nextShot = shootParams.MIDFAR;
    }

    /**
     * Moves shooter rotation to the Middle setpoint defined in constants.java
     */
    public void setWristMiddlePosition() {
        Robot.nextShot = shootParams.MIDDLE;
    }

    /**
     * Moves shooter rotation to the Mid setpoint defined in constants.java
     */
    public void setWristMidPosition() {
        Robot.nextShot = shootParams.MID;
    }

    /**
     * Moves shooter rotation to the Near setpoint defined in constants.java
     */
    public void setWristNearPosition() {
        Robot.nextShot = shootParams.NEAR;
    }

    /**
     * Moves shooter rotation to the Near setpoint defined in constants.java
     */
    public void setWristClosePosition() {
        Robot.nextShot = shootParams.CLOSE;
    }

    /**
     * Moves shooter rotation to the Wall setpoint defined in constants.java
     */
    public void setWristWallPosition() {
        Robot.nextShot = shootParams.WALL;
    }

    /**
     * Moves shooter rotation to the Wall setpoint defined in constants.java
     */
    public void setWristAmpPosition() {
        Robot.nextShot = shootParams.AMP;
    }

    public void setWristCollectPosition() {
        targetPosition = Constants.Wrist.SETPOINT_COLLECT_ABSOLUTE;
    }

    /**
     * Get the target position of the arm in absolute from 0 to 1
     * @return target position in ticks
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    public String getTargetPosition_Shuffleboard() {
        return String.format("%3.3f", targetPosition);
    }

    /**
     * Get the current error of the arm PID
     * @return the current error of the arm PID
     */
    public double getPIDWristError() {
        return getPosition() - targetPosition;
    }

    public void PIDKill() {
        pidController.setP(0.0);
        pidController.setI(0.0);
        pidController.setD(0.0);
    }

    public void PIDEnable() {
        pidController.setP(Constants.Wrist.kP);
        pidController.setI(Constants.Wrist.kI);
        pidController.setD(Constants.Wrist.kD);
    }

    public double getWristMotorTemp() {
        return motor.getMotorTemperature();
    }

    public boolean isWristCool() {
        return getWristMotorTemp() < Constants.MotorTemps.SHOOTER_MOTOR_TEMP;
    }

    /**
     *  All the config setting for Rotation (controller, sensor, rotation pid)
     */
    public void configWrist() {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.enableVoltageCompensation(12);

        motor.setOpenLoopRampRate(.25);

        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(Constants.Wrist.ENCODER_OFFSET_ABSOLUTE)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        wristEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));

        pidController = new PIDController(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD);
    }
}
