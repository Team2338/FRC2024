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
    public static SparkPIDController pidController;

    public Shooter() {
        shooter = new CANSparkMax(RobotMap.SHOOTER, CANSparkLowLevel.MotorType.kBrushless);
        shooter.restoreFactoryDefaults();
        shooter.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pidController = shooter.getPIDController();

        pidController.setP(Constants.Shooter.kP);
        pidController.setFF(Constants.Shooter.FF);
        pidController.setOutputRange(0,1);
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
        pidController.setReference(rpm, CANSparkBase.ControlType.kVelocity);
    }

    public double getRPM() {
        return shooter.getEncoder().getVelocity();
    }

    public String getRPM_Shuffleboard() {
        return String.format("%12.0f", getRPM());
    }
}
