package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Climber extends SubsystemBase {
    public static CANSparkMax climberMotor;
    public static SparkPIDController pidClimber;
    public static double targetPosition;

    public Climber() {
        climberMotor = new CANSparkMax(RobotMap.CLIMBER_ID, CANSparkLowLevel.MotorType.kBrushless);
        config();

        targetPosition = climberMotor.getEncoder().getPosition();
    }

//    public void setClimberPercent(double percent) {
//        climberMotor.set(percent);
//    }

    public void setTargetPosition(double pos) {
        targetPosition = pos;
    }

    public double getClimberPosition() {
        return climberMotor.getEncoder().getPosition();
    }

    public void resetPosition() {
        climberMotor.getEncoder().setPosition(0);
        targetPosition = 0;
    }

//    public void up(boolean slow){
//        if (slow) {
//            setClimberPercent(0.2); // +0.3 for climber, +0.25 for elevator
//        } else {
//            setClimberPercent(0.8);
//        }
//    }
//
//    public void down(boolean slow){
//        if (slow) {
//            setClimberPercent(-0.2); // -.30 for climber, +0.05 for elevator
//        } else {
//            setClimberPercent(-0.8);
//        }
//    }

    /**
     * Used to move climber in manual mode, using joystick % is input
     * @param percent
     */
    public void move(double percent) {
        // soft limits will keep the robot arm in allowable range
        climberMotor.set(percent);
    }

//    public void hold(){
//        setClimberPercent(0); // 0 for climber, +0.15 for elevator
//    }

    public void PIDHold() {
        System.out.println(targetPosition);
        climberMotor.getPIDController().setReference(targetPosition,CANSparkBase.ControlType.kPosition);
    }

    public String getClimberPosition_Shuffleboard() {
        return String.format("%12.0f", getClimberPosition()*100);
    }

    public void config() {
        climberMotor.restoreFactoryDefaults();
        climberMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        climberMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,(float) Constants.Climber.MAX_LIMIT/100);
        climberMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,(float) Constants.Climber.MIN_LIMIT/100);
        climberMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,true);
        climberMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,true);

        climberMotor.setInverted(true);

        pidClimber = climberMotor.getPIDController();
        pidClimber.setFF(Constants.Climber.FF);
        pidClimber.setP(Constants.Climber.kP);
        pidClimber.setD(Constants.Climber.kD);
        pidClimber.setI(Constants.Climber.kI);
    }

    public void enableSoftLimits(boolean enable) {
        climberMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,enable);
        climberMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,enable);
    }
}

