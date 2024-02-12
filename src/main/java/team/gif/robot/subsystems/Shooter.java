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

    private double tarPos;

    public Shooter() {
        shooter = new CANSparkMax(RobotMap.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        configMainShooter();

        shooterAngle = new CANSparkMax(RobotMap.SHOOTER_ANGLE_ID, CANSparkLowLevel.MotorType.kBrushless);
        configShooterAngle();
    }

    public void setVoltage(double volt) {
//        shooter.setVoltage(volt);
        shooter.set(volt);
    }

    public double getVoltage() {
        return shooter.getBusVoltage();
    }

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
     * move the shooter angle percent
     * @param percent
     */
    public void setAnglePercentMove(double percent) {
        shooterAngle.set(percent);
    }

    /**
     * Use PID to move the shooter angle to position
     */
    public void PIDmove() {
        pidShooterAngle.setReference(tarPos, CANSparkBase.ControlType.kPosition);
    }

    /**
     * Get the current position of the arm in ticks
     * @return arm position in ticks
     */
    public double getPosition(){
        return angleEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Set the target position of the arm in ticks
     * @param pos target position in ticks
     */
    public void setAnglePos(double pos) {
        tarPos = pos;
    }

    /**
     * Get the target position of the arm in ticks
     * @return target position in ticks
     */
    public double getTargetPos() {return tarPos;}

    /**
     * Get the current error of the arm PID
     * @return the current error of the arm PID
     */
    public double getPIDError() {
        return getPosition() - tarPos;
    }


    /**
     *  All the config setting for both ShooterAngle and MainShooter
     */
    public void configMainShooter() {
        shooter.restoreFactoryDefaults();
        shooter.setInverted(true);
        shooter.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pidMainShooter = shooter.getPIDController();

        pidMainShooter.setP(Constants.Shooter.kP);
        pidMainShooter.setFF(Constants.Shooter.FF);
        pidMainShooter.setI(Constants.Shooter.kI);
        pidMainShooter.setOutputRange(0,1);
    }

    public void configShooterAngle() {
        shooterAngle.restoreFactoryDefaults();
        shooterAngle.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shooterAngle.setOpenLoopRampRate(.25);

        shooterAngle.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,true);
        shooterAngle.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
        shooterAngle.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) Constants.Shooter.MAX_LIMIT);
        shooterAngle.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) Constants.Shooter.MIN_LIMIT);

        pidShooterAngle = shooterAngle.getPIDController();
        pidShooterAngle.setP(Constants.Shooter.ANGLE_kP);
        pidShooterAngle.setFF(Constants.Shooter.ANGLE_FF);
        pidShooterAngle.setI(Constants.Shooter.ANGLE_kI);
        pidShooterAngle.setD(Constants.Shooter.ANGLE_kD);
        pidShooterAngle.setOutputRange(-.2, -.2);

        angleEncoder = new CANcoder(RobotMap.SHOOTER_ANGLE_ENCODER_ID);
        MagnetSensorConfigs magSensorConfig = new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        magSensorConfig.withMagnetOffset(-.133);
        magSensorConfig.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magSensorConfig));
    }
}
