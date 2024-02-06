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
import team.gif.robot.RobotMap;

public class Shooter extends SubsystemBase {
    public static CANSparkMax shooter;
    public static SparkPIDController pidMainShooter;

    public static CANSparkMax shooterAngle;
    public static SparkPIDController pidShooterAngle;
    public static CANcoder angleEncoder;

    public Shooter() {
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
        pidShooterAngle.setP(Constants.Shooter.ANGLE_kP);
        pidShooterAngle.setFF(Constants.Shooter.ANGLE_FF);

        angleEncoder = new CANcoder(RobotMap.SHOOTER_ANGLE_ENCODER_ID);
        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        magSensorConfig.withMagnetOffset(-.37);
        magSensorConfig.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));
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

    // Angling
    public void setAnglePercent(double percent) {
        shooterAngle.set(percent);
    }

    public void setAnglePos(double pos) {
        pidShooterAngle.setReference(pos, CANSparkBase.ControlType.kPosition);
    }

    public double getPosition(){
        return angleEncoder.getAbsolutePosition().getValueAsDouble();
    }
}
