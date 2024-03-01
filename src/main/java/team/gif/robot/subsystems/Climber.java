package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Climber extends SubsystemBase {
    public static CANSparkMax climber;
    public static SparkPIDController pidClimber;
    public static double targetPosition;

    public Climber() {
        climber = new CANSparkMax(RobotMap.CLIMBER_ID, CANSparkLowLevel.MotorType.kBrushless);
        config();

        targetPosition = climber.getEncoder().getPosition();
    }

    public void setClimberPercent(double percent) {
        climber.set(percent);
    }

    public double getClimberPosition() {
        return climber.getEncoder().getPosition();
    }

    public void setTargetPosition(double pos) {
        targetPosition = pos;
    }

    public void resetPosition() {
        climber.getEncoder().setPosition(0);
        System.out.println("here "+ getClimberPosition());
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

    public void PIDHold() {
        climber.getEncoder().setPosition(targetPosition);
    }

    public String getClimberPosition_Shuffleboard() {
        return String.format("%12.0f", getClimberPosition()*100);
    }

    public void config() {
        climber.restoreFactoryDefaults();
        climber.setIdleMode(CANSparkBase.IdleMode.kBrake);
        climber.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,50);
        climber.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,0);
        climber.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,true);
        climber.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,true);

        climber.setInverted(true);

        pidClimber = climber.getPIDController();
        pidClimber.setFF(Constants.Climber.FF);
        pidClimber.setP(Constants.Climber.kP);
        pidClimber.setD(Constants.Climber.kD);
        pidClimber.setI(Constants.Climber.kI);
    }

    public void enableSoftLimits(boolean enable) {
        climber.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,enable);
        climber.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,enable);
    }
}

