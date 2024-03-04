package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Climber extends SubsystemBase {
    public static CANSparkMax motor;
    public static SparkPIDController pidController;
    public static double targetPosition;

    public Climber() {
        motor = new CANSparkMax(RobotMap.CLIMBER_ID, CANSparkLowLevel.MotorType.kBrushless);
        config();

        targetPosition = motor.getEncoder().getPosition();
    }

    public void setTargetPosition(double pos) {
        targetPosition = pos;
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public void resetPosition() {
        motor.getEncoder().setPosition(0);
        targetPosition = 0;
    }

    /**
     * Used to move climber in manual mode, using joystick % is input
     * @param percent
     */
    public void move(double percent) {
        // soft limits will keep the robot climber in allowable range
        motor.set(percent);
    }

    public void PIDHold() {
        motor.getPIDController().setReference(targetPosition,CANSparkBase.ControlType.kPosition);
    }

    public String getPosition_Shuffleboard() {
        return String.format("%12.3f", getPosition());
    }

    public double getMotorTemp() {
        return motor.getMotorTemperature();
    }

    public boolean isMotorCool() {
        return getMotorTemp() <= Constants.MotorTemps.CLIMBER_MOTOR_TEMP;
    }

    public void config() {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,(float) Constants.Climber.LIMIT_MAX);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,(float) Constants.Climber.LIMIT_MIN);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,true);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,true);

        motor.setInverted(true);

        pidController = motor.getPIDController();
        pidController.setFF(Constants.Climber.FF);
        pidController.setP(Constants.Climber.kP);
        pidController.setD(Constants.Climber.kD);
        pidController.setI(Constants.Climber.kI);
    }

    public void enableSoftLimits(boolean enable) {
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,enable);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,enable);
    }
}

