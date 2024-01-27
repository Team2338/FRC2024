package team.gif.robot.subsystems.drivers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import team.gif.robot.Constants;
import team.gif.robot.subsystems.SwerveDrivetrainMK3;

/**
 * @author Rohan Cherukuri
 * @since 2/14/22
 */
public class SwerveModuleMK3 {
    private final CANSparkMax driveMotor;
//    private final CANCoder canCoder;
    private final WPI_TalonSRX turnMotor;

    private final double kFF;
    private final double kP;
    private double accum = 0;

    private final boolean isAbsInverted;

    private double turningOffset;
    public double target;

    /**
     * Constructor for a TalonSRX, NEO based Swerve Module
     * @param driveMotor SparkMax (NEO) motor channel ID
     * @param turnMotor TalonFX (Falcon) motor channel ID
     * @param isTurningInverted Boolean for if the motor turning the axle is inverted
     * @param isDriveInverted Boolean for if the motor driving the wheel is inverted
     * @param isAbsInverted Boolean for if the absolute encoder checking turn position is inverted
     * @param turningOffset Difference between the absolute encoder and the encoder on the turnMotor
     */
    public SwerveModuleMK3(
            int driveMotor,
            int turnMotor,
            boolean isTurningInverted,
            boolean isDriveInverted,
            boolean isAbsInverted,
            double turningOffset,
            double kFF,
            double kP
    ) {
        //init motors
        this.driveMotor = new CANSparkMax(driveMotor, CANSparkLowLevel.MotorType.kBrushless);
        this.turnMotor = new WPI_TalonSRX(turnMotor);

        //reset
        this.driveMotor.restoreFactoryDefaults();
        this.turnMotor.configFactoryDefault();

        //stupid brake mode
        this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.turnMotor.setNeutralMode(NeutralMode.Brake);

        //TODO: Do I need this for drive motor? How do I do this with talon feedback device
//        this.turnMotor.getEncoder().setPositionConversionFactor(Constants.ModuleConstants.TURNING_ENCODER_ROT_TO_RAD);
//        this.turnMotor.getEncoder().setVelocityConversionFactor(Constants.ModuleConstants.TURNING_ENCODER_RPM_2_RAD_PER_SECOND);

        this.driveMotor.setInverted(isDriveInverted);
        this.turnMotor.setInverted(isTurningInverted);
        this.isAbsInverted = isAbsInverted;
//
//        this.canCoder = new CANCoder(canCoder);
//        this.canCoder.configFactoryDefault();
//        this.canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        //TODO: is this the right encoder? Other repo shows relative
//        this.turnMotor.setSensorPhase(true);
        this.turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.turnMotor.configSelectedFeedbackCoefficient(1);


        this.turningOffset = turningOffset;

        this.kFF = kFF;
        this.kP = kP;

    }

    /**
     * Get the Falcon driving the wheel
     * @return Returns the Falcon driving the wheel
     */
    public CANSparkMax getDriveMotor() {
        return this.driveMotor;
    }

    /**
     * Get the SparkMax turning the wheel
     * @return Returns the SparkMax turning the wheel
     */
    public WPI_TalonSRX getTurnMotor() {
        return this.turnMotor;
    }

    public double getAccum() {
        return accum;
    }

    /**
     * Get the active state of the swerve module
     * @return Returns the active state of the given swerveModule
     */
    public SwerveModuleState getState() {
        //return drive velocity and turn velocity
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(Units.degreesToRadians(getTurnVelocity())));
    }

    /**
     * Get the active drive velocity
     * @return Returns the active drive velocity as a double in RPM
     */
    public double getDriveVelocity() {
        //TODO: check with mr. k
        return driveMotor.getEncoder().getVelocity() * Constants.ModuleConstants.DRIVE_ENCODER_ROT_2_METER;
    }

    /**
     * Get the drive motor's current output
     * @return the current output as a percent
     */
    public double getDriveOutput() {
        //used for logging
        return driveMotor.getAppliedOutput();
    }

    /**
     * Get the active turn velocity
     * @return Returns the active turn velocity as a double in EncoderTicks per 100ms
     */
    public double getTurnVelocity() {
        //TODO: does this return the same type of value as the spark equivilant?
        return turnMotor.getSelectedSensorVelocity();
    }

    /**
     * Get the heading of the canCoder - will also include the offset
     * @return Returns the raw heading of the canCoder (deg)
     */
    public double getRawHeading() {
        return turnMotor.getSelectedSensorPosition();
    }

    public double encoderDegrees() {
        //The first modulo (%) pulls into the range -4095…4095,
        // the add and next modulo make it 0…4095,
        // subtracting 2048 makes it -4096…4095,
        // and the multiplication and division scale it to 180.
        return ((((getRawHeading() - turningOffset) % 4096) + 4096) % 4096 - 2048) * 45 / 512;
    }

    /**
     * Get the heading of the swerve module
     * @return Returns the heading of the module in radians as a double
     */
    public double getTurningHeading() {
        double heading = Units.degreesToRadians(encoderDegrees()) * (isAbsInverted ? -1.0: 1.0);
        heading %= 2 * Math.PI;
        return heading;
    }

    /**
     * Reset the wheels to their 0 positions
     */
    public void resetWheel() {
        final double error = getTurningHeading();
        final double kff = kFF * Math.abs(error) / error;
        final double turnOutput = kff + (kP * error);

        turnMotor.set(turnOutput);
    }

    /**
     * Find the reverse of a given angle (i.e. pi/4->7pi/4)
     * @param radians the angle in radians to reverse
     * @return the reversed angle
     */
    private double findRevAngle(double radians) {
        return (Math.PI * 2 + radians) % (2 * Math.PI) - Math.PI;
    }

    /**
     * Finds the distance in ticks between two setpoints
     * @param setpoint initial/current point
     * @param position desired position
     * @return the distance between the two point
     */
    private double getDistance(double setpoint, double position) {
        return Math.abs(setpoint - position);
    }

    /**
     * Optimize the swerve module state by setting it to the closest equivalent vector
     * @param original the original swerve module state
     * @return the optimized swerve module state
     */
    private SwerveModuleState optimizeState(SwerveModuleState original) {
        // Compute all options for a setpoint
        double position = getTurningHeading();
        double setpoint = original.angle.getRadians();
        double forward = setpoint + (2 * Math.PI);
        double reverse = setpoint - (2 * Math.PI);
        double antisetpoint = findRevAngle(setpoint);
        double antiforward = antisetpoint + (2 * Math.PI);
        double antireverse = antisetpoint - (2 * Math.PI);

        // Find setpoint option with minimum distance
        double[] alternatives = { forward, reverse, antisetpoint, antiforward, antireverse };
        double min = setpoint;
        double minDistance = getDistance(setpoint, position);
        int minIndex = -1;
        for (int i = 0; i < alternatives.length; i++) {
            double dist = getDistance(alternatives[i], position);
            if (dist < minDistance) {
                min = alternatives[i];
                minDistance = dist;
                minIndex = i;
            }
        }

        // Figure out the speed. Anti- directions should be negative.
        double speed = original.speedMetersPerSecond;
        if (minIndex > 1) {
            speed *= -1;
        }

        return new SwerveModuleState(speed, new Rotation2d(min));
    }

    /**
     * Set the desired state of the swerve module
     * @param state The desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState stateOptimized = optimizeState(state);
        double driveOutput = stateOptimized.speedMetersPerSecond / SwerveDrivetrainMK3.getDrivePace().getValue();
        final double error = getTurningHeading() - stateOptimized.angle.getRadians();
        System.out.println(error);
        target = stateOptimized.angle.getRadians();
        final double kff = kFF * Math.abs(error) / error;
        //accum += error;
        final double turnOutput = kff + (kP * error) + (0.001 * accum);

        driveMotor.set(driveOutput);
        turnMotor.set(turnOutput);

    }

    /**
     * Stop the swerve modules
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Get the position of the swerve module - TODO: HAS BUG
     * @return the position of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition() * Constants.ModuleConstants.DRIVE_ENCODER_ROT_2_METER, new Rotation2d(getTurningHeading()));
    }

    /**
     * Resets the drive encoder
     */
    public void resetDriveEncoders() {
        driveMotor.getEncoder().setPosition(0.0);
    }
}
