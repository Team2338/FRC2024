package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Climber extends SubsystemBase {
    public static CANSparkMax climber;

    public Climber() {
        climber = new CANSparkMax(RobotMap.CLIMBER_ID, CANSparkLowLevel.MotorType.kBrushless);
        climber.restoreFactoryDefaults();
        climber.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void setClimberPercent(double percent) {
        climber.set(percent);
    }

    public double getClimberPosition() {
        return climber.getEncoder().getPosition();
    }

    public void up(){
        setClimberPercent(0.3); // +0.3 for climber, +0.25 for elevator
    }

    public void down(){
        setClimberPercent(-0.3); // -.30 for climber, +0.05 for elevator
    }

    public void hold(){
        setClimberPercent(0); // 0 for climber, +0.15 for elevator
    }

    public String getClimberPosition_Shuffleboard() {
        return String.format("%12.0f", getClimberPosition());
    }
}

