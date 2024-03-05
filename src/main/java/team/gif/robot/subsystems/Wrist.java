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
    public static CANSparkMax wristController;
    public static PIDController pidWrist;
    public static CANcoder wristEncoder;

    private double targetPosition; // Rotation

    public Wrist() throws Exception {
        wristController = new CANSparkMax(RobotMap.WRIST_ID, CANSparkLowLevel.MotorType.kBrushless);

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
    public void moveWristPercentPower(double percent) {
        if(getPosition() < Constants.Wrist.KILL_LIMIT_ABSOLUTE) {
            wristController.set(percent);
        } else {
            wristController.set(0);
        }
    }

    /**
     * Holds thr shooter rotation using FF <br
     */
    public void holdWrist() {
        wristController.set(Constants.Wrist.FF);
    }

    /**
     * Use PID to move the shooter angle to absolute position between 0 and 1
     */
    public void PIDWristMove() {
        double pidOutput = pidWrist.calculate(wristEncoder.getAbsolutePosition().getValueAsDouble(), targetPosition);
        wristController.set(pidOutput + Constants.Wrist.FF);
    }

    /**
     * Get the current position of the arm in absolute between 0 and 1
     * @return arm position in ticks
     */
    public double getPosition(){
        return wristEncoder.getAbsolutePosition().getValueAsDouble();
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
     * Moves shooter rotation to the Far setpoint defined in constants.java
     */
    public void setWristFarPosition() {
//        targetPosition = Constants.Wrist.SETPOINT_FAR_ABSOLUTE;
        Robot.nextShot = shootParams.FAR;
    }

    /**
     * Moves shooter rotation to the Mid setpoint defined in constants.java
     */
    public void setWristMidPosition() {
//        targetPosition = Constants.Wrist.SETPOINT_MID_ABSOLUTE;
        Robot.nextShot = shootParams.MID;
    }

    /**
     * Moves shooter rotation to the Near setpoint defined in constants.java
     */
    public void setWristNearPosition() {
//        targetPosition = Constants.Wrist.SETPOINT_NEAR_ABSOLUTE;
        Robot.nextShot = shootParams.NEAR;
    }

    /**
     * Moves shooter rotation to the Wall setpoint defined in constants.java
     */
    public void setWristWallPosition() {
//        targetPosition = Constants.Wrist.SETPOINT_WALL_ABSOLUTE;
        Robot.nextShot = shootParams.WALL;
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

    /**
     * Get the current error of the arm PID
     * @return the current error of the arm PID
     */
    public double getPIDWristError() {
        return getPosition() - targetPosition;
    }

    public void PIDKill() {
        pidWrist.setP(0.0);
        pidWrist.setI(0.0);
        pidWrist.setD(0.0);
    }

    public void PIDEnable() {
        pidWrist.setP(Constants.Wrist.kP);
        pidWrist.setI(Constants.Wrist.kI);
        pidWrist.setD(Constants.Wrist.kD);
    }

    public double getWristMotorTemp() {
        return wristController.getMotorTemperature();
    }

    public boolean isWristCool() {
        if (getWristMotorTemp() >= Constants.MotorTemps.SHOOTER_MOTOR_TEMP) {
            return false;
        }
        return true;
    }

    /**
     *  All the config setting for Rotation (controller, sensor, rotation pid)
     */
    public void configWrist() {
        wristController.restoreFactoryDefaults();
        wristController.setIdleMode(CANSparkBase.IdleMode.kBrake);
        wristController.setOpenLoopRampRate(.25);

        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(Constants.Wrist.ENCODER_OFFSET_ABSOLUTE)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        wristEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));

        pidWrist = new PIDController(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD);
    }
}
