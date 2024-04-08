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
import edu.wpi.first.wpilibj.Timer;
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

    // Stores the last good value of the wrist estimator. This is retained
    // in case the limelight loses the target. The value is retained for a
    // specified amount of time
    private double lastWristEstimateDegrees = Constants.Wrist.MIN_LIMIT_DEGREES;
    private double lastWristEstimateTimestamp = 0;

    // Other subsystems need to know if the bot is using limelight or user selected wrist angle
    private boolean autoAngleEnabled;

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

        autoAngleEnabled = false;
    }

    public String getWristDegrees_Shuffleboard() {
        return String.format("%4.1f", absoluteToDegrees(getPosition()));
    }

    /**
     * Rotates shooter wrist <br>
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
        double angle = absoluteToDegrees(getPosition());
        double ff = wristFeedForward(angle);

        motor.set(ff);
    }

    /**
     * Use PID to move the wrist to the target position between 0 and 1
     */
    public void PIDWristMove() {
        double currentAngle = getPosition();

        double pidOutput = pidController.calculate(currentAngle, targetPosition);
        double percent = pidOutput + wristFeedForward(absoluteToDegrees(currentAngle));

        // The kP is tuned high to get to the position quickly, but need to set a max and min
        // so the wrist doesn't go too quickly and overshoot the target by a lot
        percent = Math.min(percent,0.13); // this MAXIMIZES percent to 0.13 sp we don't move too fast
        percent = Math.max(percent,-0.04); // this MINIMIZES percent to -0.04 so we don't move too fast

        motor.set(percent);
    }

    /**
     * Calculate the rotational feedforward based on the angle of the wrist <br>
     * Uses a linear output instead of a curve but should be sufficient <br>
     * <br>
     * 0 is shooting straight up <br>
     * 90 is shooting straight out <br>
     *
     * @param angle degrees
     * @return feed forward
     */

     //  0
     //  ^
     //  |
     //  |---> 90
     //
    private double wristFeedForward(double angle) {
        double feedForward = Constants.Wrist.FF * (1 - (Math.abs(angle - 90) / 90));

        // at very low angles, need to apply a minimum
        feedForward = Math.max(feedForward,0.02);

        return feedForward;
    }

    public boolean isWristWithinTolerance() {
        return pidController.atSetpoint();
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
     * Sets the target position to the Far3 setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristFar3Position() {
        Robot.nextShot = shootParams.FAR3;
    }

    /**
     * Sets the target position to the Far2 setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristFar2Position() {
        Robot.nextShot = shootParams.FAR2;
    }

    /**
     * Sets the target position to the MidMidFar setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristMidMidFarPosition() {
        Robot.nextShot = shootParams.MIDMIDFAR;
    }

    /**
     * Sets the target position to the Far setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristFarPosition() {
        Robot.nextShot = shootParams.FAR;
    }

    /**
     * Sets the target position to the MidFar setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristMidFarPosition() {
        Robot.nextShot = shootParams.MIDFAR;
    }

    /**
     * Sets the target position to the Middle setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristMiddlePosition() {
        Robot.nextShot = shootParams.MIDDLE;
    }

    /**
     * Sets the target position to the Mid setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristMidPosition() {
        Robot.nextShot = shootParams.MID;
    }

    /**
     * Sets the target position to the Near setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristNearPosition() {
        Robot.nextShot = shootParams.NEAR;
    }

    /**
     * Sets the target position to the Close setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristClosePosition() {
        Robot.nextShot = shootParams.CLOSE;
    }

    /**
     * Sets the target position to the Wall setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristWallPosition() {
        Robot.nextShot = shootParams.WALL;
    }

    /**
     * Sets the target position to the Amp setpoint defined in constants.java. Does not ove wrist.
     */
    public void setWristAmpPosition() {
        Robot.nextShot = shootParams.AMP;
    }

    /**
     * Sets the target position to the Amp setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristTrapPosition() {
        Robot.nextShot = shootParams.TRAP;
    }

    /**
     * Sets the target position to the Collect setpoint defined in constants.java. This method moves the wrist (indirectly).
     */
    public void setWristCollectPosition() {
        targetPosition = Constants.Wrist.SETPOINT_COLLECT_ABSOLUTE;
    }

    /**
     * Sets the target position of the wrist in ticks based on the Limelight
     * wrist angle estimator method. This method moves the wrist (indirectly).
     */
    public void setWristAuto() {
        Robot.nextShot = shootParams.AUTOSHOT;
        targetPosition = degreesToAbsolute(wristEstimatorDegrees());
    }

    public void enableAutoAngle() {
        autoAngleEnabled = true;
    }

    public void disableAutoAngle() {
        autoAngleEnabled = false;
    }

    public boolean isAutoAngleEnabled() {
        return autoAngleEnabled;
    }

    /**
     * Estimate the wrist angle in degrees based on the distance to the target
     * @return wrist angle in degrees <br>
     *          returns wrist minimum if limelight lock is too stale (1 second)
     */
    public double wristEstimatorDegrees() {
        double timestamp = Timer.getFPGATimestamp();
        double distance = Robot.limelightShooter.getDistance();
        double angle;

        //If the limelight has no target
        if (distance == -1) {
            //The last wrist estimate is expired. Set to wrist minimum.
            if (timestamp - lastWristEstimateTimestamp > 1.0) { // allow keep previous value for 1 second
                lastWristEstimateDegrees = Constants.Wrist.MIN_LIMIT_DEGREES;
            }

            // use the angle from the last good value
            angle = lastWristEstimateDegrees;
        } else {
            // limelight has a target
            // 43.38 is the closest the robot can get to the target
            // the farther away the robot gets the lower the angle due to gravity (use distance ratio)
            distance = 0.80*(distance - 43.38) + 43.38;

            // 58 is the height between the limelight lens and target in inches
            // atan is in radians, convert to degrees using 180/pi
            angle = 90 - ((Math.atan(58/distance))*(180 / 3.14159));

            // store this as last good value, along with timestamp
            lastWristEstimateDegrees = angle;
            lastWristEstimateTimestamp = timestamp;
        }

        return angle;
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

        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(Constants.Wrist.ENCODER_OFFSET_ABSOLUTE)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        wristEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));

        pidController = new PIDController(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD);
        pidController.setTolerance(1.5* Constants.Wrist.ABSOLUTE_PER_DEGREE);
    }
}
