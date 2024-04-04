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

    // Value of the wrist encoder from 0 to 1. Given the gear setup, the
    // encoder wraps around 3 times in a full circle
    // i.e. when turning the wrist, it goes from 0->1,0->1,0->1 in a full rotation
    private double targetPosition;

    public Wrist() throws Exception {
        motor = new CANSparkMax(RobotMap.WRIST_ID, CANSparkLowLevel.MotorType.kBrushless);

        wristEncoder = new CANcoder(RobotMap.WRIST_ENCODER_ID);
        configWrist();

        // check constants are valid
        if (Constants.Wrist.MIN_LIMIT_ABSOLUTE < Constants.Wrist.HARD_STOP_ABSOLUTE){
            throw new Exception("Shooter MIN_LIMIT_ABSOLUTE needs to be greater than HARD_STOP_ABSOLUTE");
        }

        // set the initial wrist position so PID can hold the current position when first enabled
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
     * Holds the wrist using FF <br
     */
    public void holdWrist() {
        motor.set(Constants.Wrist.FF);
    }

    /**
     * Use PID to move the wrist to an absolute position between 0 and 1
     */
    public void PIDWristMove() {
        double pidOutput = pidController.calculate(wristEncoder.getAbsolutePosition().getValueAsDouble(), targetPosition);
        double percent = pidOutput + Constants.Wrist.FF;

        // The kP ws tuned high to get to the position quickly, but need to set a max and min
        // so the wrist doesn't go too quickly and overshoot the target by a lot
        percent = Math.min(percent,0.13); // this MAXIMIZES percent to 0.13 sp we don't move too fast
        percent = Math.max(percent,-0.04); // this MINIMIZES percent to -0.04 so we don't move too fast

        // formulas get the wrist to within 5 degrees but the wrist needs a little more power to get to the setpoint
        // only adjust for shots other than the Amp (and Trap doesn't use pid)
        if (getTargetPosition() < Constants.Wrist.SETPOINT_AMP_ABSOLUTE ) {
            if (Math.abs(pidController.getPositionError()) < (Constants.Wrist.ABSOLUTE_PER_DEGREE*5)) {
                if (Math.abs(pidController.getPositionError()) > (Constants.Wrist.ABSOLUTE_PER_DEGREE * 0.6)) {
                    if (percent > 0) {
                        percent += (1.3*Constants.Wrist.FF); // 1.0 and 1.3  works pretty well, 1.7 shakes
                    }
                }
            }
        }

        percent = Math.min(percent,0.13);
        percent = Math.max(percent,-0.04);
        motor.set(percent);
    }

    /**
     * Get the current position of the wrist in absolute between 0 and 1
     * @return wrist position in ticks
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

    /**
     * Converts from degrees to wrist encoder absolute ticks <br>
     * 0 is wrist pointing straight up<br>
     * 90 is wrist pointing straight out of the bot <br>
     *
     * @param degrees desired degree
     * @return absolute ticks from 0 to 1
     */
    public double degreesToAbsolute(double degrees){
        return (degrees - Constants.Wrist.MIN_LIMIT_DEGREES) * Constants.Wrist.ABSOLUTE_PER_DEGREE + Constants.Wrist.MIN_LIMIT_ABSOLUTE;
    }

    /**
     * Converts from wrist encoder absolute ticks to degrees <br>
     * 0 is wrist pointing straight up <br>
     * 90 is wrist pointing straight out of the bot <br>
     *
     * @param absolute ticks from 0 to 1
     * @return degrees desired degree
     */
    public double absoluteToDegrees(double absolute){
        return ( (absolute - Constants.Wrist.MIN_LIMIT_ABSOLUTE)/ Constants.Wrist.ABSOLUTE_PER_DEGREE +  Constants.Wrist.MIN_LIMIT_DEGREES);
    }

    /**
     * Set the target position of the wrist in absolute from 0 to 1 <br>
     * With PID running, this sets the new position and PID will make the
     * wrist move
     * @param pos target position in ticks
     */
    public void setTargetPosition(double pos) {
        targetPosition = pos;
    }

    /**
     * Moves wrist to the Far3 setpoint defined in constants.java
     */
    public void setWristFar3Position() {
        Robot.nextShot = shootParams.FAR3;
    }

    /**
     * Moves wrist to the Far2 setpoint defined in constants.java
     */
    public void setWristFar2Position() {
        Robot.nextShot = shootParams.FAR2;
    }

    /**
     * Moves wrist to the MidMidFar setpoint defined in constants.java
     */
    public void setWristMidMidFarPosition() {
        Robot.nextShot = shootParams.MIDMIDFAR;
    }

    /**
     * Moves wrist to the Far setpoint defined in constants.java
     */
    public void setWristFarPosition() {
        Robot.nextShot = shootParams.FAR;
    }

    /**
     * Moves wrist to the MidFar setpoint defined in constants.java
     */
    public void setWristMidFarPosition() {
        Robot.nextShot = shootParams.MIDFAR;
    }

    /**
     * Moves wrist to the Middle setpoint defined in constants.java
     */
    public void setWristMiddlePosition() {
        Robot.nextShot = shootParams.MIDDLE;
    }

    /**
     * Moves wrist to the Mid setpoint defined in constants.java
     */
    public void setWristMidPosition() {
        Robot.nextShot = shootParams.MID;
    }

    /**
     * Moves wrist to the Near setpoint defined in constants.java
     */
    public void setWristNearPosition() {
        Robot.nextShot = shootParams.NEAR;
    }

    /**
     * Moves wrist to the Close setpoint defined in constants.java
     */
    public void setWristClosePosition() {
        Robot.nextShot = shootParams.CLOSE;
    }

    /**
     * Moves wrist to the Wall setpoint defined in constants.java
     */
    public void setWristWallPosition() {
        Robot.nextShot = shootParams.WALL;
    }

    /**
     * Moves wrist to the Amp setpoint defined in constants.java
     */
    public void setWristAmpPosition() {
        Robot.nextShot = shootParams.AMP;
    }

    /**
     * Moves wrist to the Collect setpoint defined in constants.java
     */
    public void setWristCollectPosition() {
        targetPosition = Constants.Wrist.SETPOINT_COLLECT_ABSOLUTE;
    }

    /**
     * Sets the new target position of the wrist in ticks based on the wrist angle estimator method
     */
    public void setWristAuto() {
        targetPosition = degreesToAbsolute(wristEstimatorDegrees());
    }

    /**
     * Estimate the wrist angle in degrees based on the distance to the target
     * @return wrist angle in degrees
     */
    public double wristEstimatorDegrees() {
        double distance = Robot.limelightShooter.DistanceEstimator(35,16.50,57);
        distance = (distance - 43.38)*.8 + 43.38;; // the farther away the robot gets the lower the angle due to gravity
        return 90 - ((Math.atan(58/distance))*(180 / 3.14159));
    }

    /**
     * Get the target position of the wrist in absolute from 0 to 1
     * @return target position in ticks
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Formats the target position for teh shuffleboard for easier ability to read
     * @return target position in ticks
     */
    public String getTargetPosition_Shuffleboard() {
        return String.format("%3.3f", targetPosition);
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

//        motor.setOpenLoopRampRate(.25);

        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(Constants.Wrist.ENCODER_OFFSET_ABSOLUTE)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        wristEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));

        pidController = new PIDController(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD);
        pidController.setTolerance(Constants.Wrist.ABSOLUTE_PER_DEGREE*5);
    }
}
