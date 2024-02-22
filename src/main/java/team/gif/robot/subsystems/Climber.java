package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public String getClimberPosition_Shuffleboard() {
        return String.format("%12.0f", getClimberPosition());
    }
}
