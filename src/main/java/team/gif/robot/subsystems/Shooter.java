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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class Shooter extends SubsystemBase {
    public static CANSparkMax shooter;
    public static SparkPIDController pidMainShooter;

    public static CANSparkMax shooterAngle;
    public static SparkPIDController pidShooterAngle;
    public static CANcoder angleEncoder;

    public Shooter() throws Exception {
        shooter = new CANSparkMax(RobotMap.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        shooter.restoreFactoryDefaults();
        shooter.setInverted(true);
        shooter.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pidMainShooter = shooter.getPIDController();

        pidMainShooter.setP(Constants.Shooter.kP);
        pidMainShooter.setFF(Constants.Shooter.FF);
        pidMainShooter.setI(Constants.Shooter.kI);
        pidMainShooter.setOutputRange(0,1);

        shooterAngle = new CANSparkMax(RobotMap.SHOOTER_ANGLE_ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterAngle.restoreFactoryDefaults();
        shooterAngle.setIdleMode(CANSparkBase.IdleMode.kBrake);

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

    // Shooter RPM
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
     * negative (-) value is counterclockwise (shoots higher)
     *
     * @param percent  The percentage of motor power
     */
    public void moveAnglePercentPower(double percent) {
        shooterAngle.set(percent);
    }

    public void setAnglePos(double pos) {
        pidShooterAngle.setReference(pos, CANSparkBase.ControlType.kPosition);
    }

    public double getPosition(){
        return angleEncoder.getAbsolutePosition().getValueAsDouble();
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
}
