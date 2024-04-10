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

    /**
     * Currently set to 1.5 degrees of tolerance (set by the pidController setTolerance method)
     *
     * @return true if wrist is within tolerance
     */
    public boolean isWristWithinTolerance() {
//        return pidController.atSetpoint();
        double tolerance = 1.5*Constants.Wrist.ABSOLUTE_PER_DEGREE;
        double delta = Math.abs(getPosition() - targetPosition);
//        System.out.println("wrist delta: " + delta + "tolerance " + tolerance);
        return delta < tolerance;
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
     * Sets the next shot parameters for the Far3 setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristFar3Position() {
        Robot.nextShot = shootParams.FAR3;
    }

    /**
     * Sets the next shot parameters for the Far2 setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristFar2Position() {
        Robot.nextShot = shootParams.FAR2;
    }

    /**
     * Sets the next shot parameters for the MidMidFar setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristMidMidFarPosition() {
        Robot.nextShot = shootParams.MIDMIDFAR;
    }

    /**
     * Sets the next shot parameters for the Far setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristFarPosition() {
        Robot.nextShot = shootParams.FAR;
    }

    /**
     * Sets the next shot parameters for the MidFar setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristMidFarPosition() {
        Robot.nextShot = shootParams.MIDFAR;
    }

    /**
     * Sets the next shot parameters for the Middle setpoint defined in constants.java. Does not move wrist.
     */
    public void setWristMiddlePosition() {
        Robot.nextShot = shootParams.MIDDLE;
    }

    /**
     * Sets the next shot parameters for the Mid setpoint defined in constants.java. Does not move wrist.
     */
    public void setNextShotMid() {
        Robot.nextShot = shootParams.MID;
    }

    /**
     * Sets the next shot parameters for the Near setpoint defined in constants.java. Does not move wrist.
     */
    public void setNextShotNear() {
        Robot.nextShot = shootParams.NEAR;
    }

    /**
     * Sets the next shot parameters for the Close setpoint defined in constants.java. Does not move wrist.
     */
    public void setNextShotClose() {
        Robot.nextShot = shootParams.CLOSE;
    }

    /**
     * Sets the next shot parameters for the Wall setpoint defined in constants.java. Does not move wrist.
     */
    public void setNextShotWall() {
        Robot.nextShot = shootParams.WALL;
    }

    /**
     * Sets the next shot parameters for the Amp setpoint defined in constants.java. Does not move wrist.
     */
    public void setNextShotAmp() {
        Robot.nextShot = shootParams.AMP;
    }

    /**
     * Sets the next shot parameters for the Trap setpoint defined in constants.java. Does not move wrist.
     */
    public void setNextShotTrap() {
        Robot.nextShot = shootParams.TRAP;
    }

    /**
     * Sets the target position to the Collect setpoint defined in constants.java. This method moves the wrist (indirectly).
     */
    public void setWristCollectPosition() {
        targetPosition = Constants.Wrist.SETPOINT_COLLECT_ABSOLUTE;
    }

    /**
     * Sets the next shot parameter to Auto and updates the Robot auto params (wrist angle
     * position in ticks, RPM and PID values) based on the Limelight wrist angle estimator
     * method. Sets auto params dirty flag if params (RPM,PID) changed from last call. Does not move the wrist.
     */
    public void setNextShotAuto() {
//        shootParams priorAuto; // used for debugging
        Robot.nextShot = shootParams.AUTO;

        double distance = Robot.limelightShooter.getDistance();
        Robot.autoWristAngleAbs = degreesToAbsolute(wristEstimatorDegrees(distance));

//        priorAuto = Robot.autoType;
        // setting up next shot
        if (distance < 0) { // there was an error getting the distance, set it to the farthest shot
            if (Robot.autoType != shootParams.FAR3) {
                Robot.autoParamsDirtyFlag = true;
                Robot.autoType = shootParams.FAR3;
                Robot.autoShooterRPM = Constants.Shooter.RPM_FAR3;
                Robot.autoShooterMinRPM = Constants.Shooter.RPM_MIN_FAR3;
                Robot.autoShooterFF = Constants.Shooter.FF_FAR3;
            }
        } else if (distance < 4*12) { // roughly wall (4')
            if (Robot.autoType != shootParams.WALL) {
                Robot.autoParamsDirtyFlag = true;
                Robot.autoType = shootParams.WALL;
                Robot.autoShooterRPM = Constants.Shooter.RPM_WALL;
                Robot.autoShooterMinRPM = Constants.Shooter.RPM_MIN_WALL;
                Robot.autoShooterFF = Constants.Shooter.FF_WALL;
            }
        } else if (distance < 12*12) { // roughly near (8')
            if (Robot.autoType != shootParams.NEAR) {
                Robot.autoParamsDirtyFlag = true;
                Robot.autoType = shootParams.NEAR;
                Robot.autoShooterRPM = Constants.Shooter.RPM_NEAR;
                Robot.autoShooterMinRPM = Constants.Shooter.RPM_MIN_NEAR;
                Robot.autoShooterFF = Constants.Shooter.FF_NEAR;
            }
        } else if (distance < 15*12) { // roughly mid (10')
            if (Robot.autoType != shootParams.MID) {
                Robot.autoParamsDirtyFlag = true;
                Robot.autoType = shootParams.MID;
                Robot.autoShooterRPM = Constants.Shooter.RPM_MID;
                Robot.autoShooterMinRPM = Constants.Shooter.RPM_MIN_MID;
                Robot.autoShooterFF = Constants.Shooter.FF_MID;
            }
        } else {
            if (Robot.autoType != shootParams.FAR3) {
                Robot.autoParamsDirtyFlag = true;
                Robot.autoType = shootParams.FAR3;
                Robot.autoShooterRPM = Constants.Shooter.RPM_FAR3;
                Robot.autoShooterMinRPM = Constants.Shooter.RPM_MIN_FAR3;
                Robot.autoShooterFF = Constants.Shooter.FF_FAR3;
            }
        }
    }

    /**
     * Sets the next shot parameter to Auto and updates the Robot auto params (wrist angle
     * position in ticks, RPM and PID values) based on the Limelight wrist angle estimator
     * method. Sets auto params dirty flag if params (RPM,PID) changed from last call. This
     * method moves the wrist (indirectly).
     */
    public void setWristAuto() {
        setNextShotAuto();
        targetPosition = Robot.autoWristAngleAbs;
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
     * calculated by the Limelight
     * @return wrist angle in degrees <br>
     *          returns wrist minimum if limelight lock is too stale (1 second)
     */
    public double wristEstimatorDegrees() {
        return wristEstimatorDegrees(Robot.limelightShooter.getDistance());
    }

    /**
     * Estimate the wrist angle in degrees based on the provided distance to the target
     * @param distanceInches distance to the target in inches
     * @return wrist angle in degrees <br>
     *          returns wrist minimum if limelight lock is too stale (1 second)
     */
    public double wristEstimatorDegrees(double distanceInches) {
        double timestamp = Timer.getFPGATimestamp();
        double angle;
        double distance = distanceInches;
        double robotAngle = Math.abs(Robot.pigeon.get180Heading());

        // If the limelight doesn't have a target
        if (distance == -1) {
            //The last wrist estimate is expired. Set to wrist minimum.
            if (timestamp - lastWristEstimateTimestamp > 1.0) { // allow keep previous value for 1 second
                lastWristEstimateDegrees = Constants.Wrist.MIN_LIMIT_DEGREES;
                System.out.println("Wrist degree estimator too stale");
            }

            // use the angle from the last good value
            angle = lastWristEstimateDegrees;
        } else {
            // limelight has a target
            // 43.38 is the closest the robot can get to the target
            // the farther away the robot gets the lower the angle due to gravity (use distance ratio)
            if (robotAngle < 10) {
                distance = 0.8 * (distance - 43.38) + 43.38;
//                System.out.println("angle " + robotAngle + " gain 0.8");
            } else {
                distance = (0.8 - robotAngle * 0.0006600 )* (distance - 43.38) + 43.38;
//                System.out.println("angle " + robotAngle + " gain " + (0.8 - robotAngle * 0.0006600 ));
            }

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
