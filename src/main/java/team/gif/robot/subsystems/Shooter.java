package team.gif.robot.subsystems;

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

    public Shooter() {
        shooter = new CANSparkMax(RobotMap.SHOOTER, CANSparkLowLevel.MotorType.kBrushless);
        shooter.restoreFactoryDefaults();
        shooter.setInverted(true);
        shooter.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pidMainShooter = shooter.getPIDController();

        pidMainShooter.setP(Constants.Shooter.kP);
        pidMainShooter.setFF(Constants.Shooter.FF);
        pidMainShooter.setOutputRange(0,1);

        shooterAngle = new CANSparkMax(RobotMap.SHOOTER_ANGLE, CANSparkLowLevel.MotorType.kBrushless);
        shooterAngle.restoreFactoryDefaults();
        shooterAngle.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pidShooterAngle = shooterAngle.getPIDController();
        pidShooterAngle.setP(Constants.Shooter.ANGLE_kP);
        pidShooterAngle.setFF(Constants.Shooter.ANGLE_FF);
    }

    public void setVoltage(double volt) {
//        shooter.setVoltage(volt);
        shooter.set(volt);
    }

    public double getVoltage() {
        return shooter.getBusVoltage();
    }

    // RPM
    public void setRPM(double rpm) {
        pidMainShooter.setReference(rpm, CANSparkBase.ControlType.kVelocity);
    }

    public double getRPM() {
        return shooter.getEncoder().getVelocity();
    }

    public String getRPM_Shuffleboard() {
        return String.format("%12.0f", getRPM());
    }

    // Angling
    public void setAnglePercent(double percent) {
        shooterAngle.set(percent);
    }

    public double getAngleEncoder(){
        return shooterAngle.getEncoder().getPosition();
    }

    public void setAnglePos(double pos) {
        pidShooterAngle.setReference(pos, CANSparkBase.ControlType.kPosition);
    }
}
