package team.gif.robot.subsystems.drivers;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import team.gif.robot.Constants;
import team.gif.robot.subsystems.SwerveDrivetrain;

/**
 * @author Rohan Cherukuri
 * @since 2/14/22
 */
public class SwerveModuleMK4 {
    private final TalonFX driveMotor;
    private final CANcoder canCoder;
    private final CANSparkMax turnMotor;

    private final double kFF;
    private final double kP;
    private double accum = 0;

    private final boolean isAbsInverted;

    private double turningOffset;

    /**
     * Constructor for a TalonSRX, NEO based Swerve Module
     * @param driveMotor SparkMax (NEO) motor channel ID
     * @param turnMotor TalonFX (Falcon) motor channel ID
     * @param isTurningInverted Boolean for if the motor turning the axle is inverted
     * @param isDriveInverted Boolean for if the motor driving the wheel is inverted
     * @param isAbsInverted Boolean for if the absolute encoder checking turn position is inverted
     * @param turningOffset Difference between the absolute encoder and the encoder on the turnMotor
     */
    public SwerveModuleMK4(
            int driveMotor,
            int turnMotor,
            boolean isTurningInverted,
            boolean isDriveInverted,
            boolean isAbsInverted,
            double turningOffset,
            int canCoder,
            double kFF,
            double kP
    ) {
        this.driveMotor = new TalonFX(driveMotor);
        this.turnMotor = new CANSparkMax(turnMotor, CANSparkLowLevel.MotorType.kBrushless);//CANSparkMaxLowLevel.MotorType.kBrushless);

        this.driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        this.turnMotor.restoreFactoryDefaults();

        this.driveMotor.setNeutralMode(NeutralModeValue.Brake); //NeutralMode.Brake);
        this.turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.turnMotor.getEncoder().setPositionConversionFactor(Constants.ModuleConstants.TURNING_ENCODER_ROT_TO_RAD);
        this.turnMotor.getEncoder().setVelocityConversionFactor(Constants.ModuleConstants.TURNING_ENCODER_RPM_2_RAD_PER_SECOND);

        this.driveMotor.setInverted(isDriveInverted);
        this.turnMotor.setInverted(isTurningInverted);
        this.isAbsInverted = isAbsInverted;

        this.canCoder = new CANcoder(canCoder);
        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        this.canCoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));
//        this.canCoder.configAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf); //AbsoluteSensorRange.Signed_PlusMinus180);

        this.turnMotor.setSmartCurrentLimit(70, 50);

        this.turningOffset = turningOffset;

        this.kFF = kFF;
        this.kP = kP;

    }

    /**
     * Get the Falcon driving the wheel
     * @return Returns the Falcon driving the wheel
     */
    public TalonFX getDriveMotor() {
        return this.driveMotor;
    }

    /**
     * Get the temp from the drive motor
     * @return the temperature of the drive motor
     */
    public double getDriveTemp() {
        return this.driveMotor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * Get the SparkMax turning the wheel
     * @return Returns the SparkMax turning the wheel
     */
    public CANSparkMax getTurnMotor() {
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
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(Units.degreesToRadians(getTurnVelocity())));
    }

    /**
     * Get the active drive velocity
     *
     * @return Returns the active drive velocity as a double in RPM
     */
    public double getDriveVelocity() {
//        return driveMotor.getSelectedSensorVelocity() * Constants.ModuleConstants.DRIVE_ENCODER_ROT_2_METER; //old code.
        return driveMotor.getMotorVoltage().getValueAsDouble() * Constants.ModuleConstants.DRIVE_ENCODER_ROT_2_METER;
    }

    /**
     * Get the drive motor's current output
     * @return the current output as a percent
     */
    public double getDriveOutput() {
        return driveMotor.get();
    }

    /**
     * Get the active turn velocity
     *
     * @return Returns the active turn velocity as a double in EncoderTicks per 100ms
     */
    public double getTurnVelocity() {
        return canCoder.getVelocity().getValueAsDouble();
    }

    public double encoderDegrees() {
        return getRawHeading() * 360;
    }

    /**
     * Get the heading of the canCoder - will also include the offset
     *
     * @return Returns the raw heading of the canCoder (deg)
     */
    public double getRawHeading() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Get the heading of the swerve module
     * @return Returns the heading of the module in radians as a double
     */
    public double getTurningHeading() {
        double heading = Units.degreesToRadians(encoderDegrees() - turningOffset) * (isAbsInverted ? -1.0: 1.0); //turningOffset
        heading %= 2 * Math.PI;
        return heading;
    }

    /**
     * Get the heading of the swerve module
     * @return Returns the heading of the module in degrees as a double
     */
    public double getTurningHeadingDegrees() {
        double heading = (encoderDegrees() - turningOffset) * (isAbsInverted ? -1.0: 1.0);
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
        double driveOutput = stateOptimized.speedMetersPerSecond / SwerveDrivetrain.getDrivePace().getValue();
        final double error = getTurningHeading() - stateOptimized.angle.getRadians();
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
     *
     * Process:
     * Set adjust to 1.0
     * Create auto with the longest straight line possible
     * adjust = actual distance / desired distance
     *
     */
    public SwerveModulePosition getPosition() {
        double adjust = 0.9578661376;
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() * Constants.ModuleConstants.DRIVE_ENCODER_ROT_2_METER  * 2176.5 * adjust, new Rotation2d(getTurningHeading()));
    }

    /**
     * Resets the drive encoder
     */
    public void resetDriveEncoders() {
        driveMotor.setPosition(0.0);
//        driveMotor.setSelectedSensorPosition(0.0); //old code
    }

    public boolean isDriveMotorCool() {
        if (getDriveTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP) {
            return false;
        }
        return true;
    }
}
