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

    public double getMotorPercent() {
        return motor.getAppliedOutput();
    }

    public void PIDHold(int slotId) {
        motor.getPIDController().setReference(targetPosition,CANSparkBase.ControlType.kPosition, slotId);
    }

    public String getPosition_Shuffleboard() {
        return String.format("%3.3f", getPosition());
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

        // PID Slots:
        // 0: Default (prevent moving during match)
        // 1: Climb Mode
        pidController = motor.getPIDController();

        pidController.setFF(Constants.Climber.FFHold, 0);
        pidController.setP(Constants.Climber.kPHold, 0);
        pidController.setI(Constants.Climber.kIHold, 0);
        pidController.setD(Constants.Climber.kDHold, 0);

        pidController.setFF(Constants.Climber.FFClimb, 1);
        pidController.setP(Constants.Climber.kPClimb,1);
        pidController.setD(Constants.Climber.kDClimb, 1);
        pidController.setI(Constants.Climber.kIClimb, 1);
    }

    public void enableSoftLimits(boolean enable) {
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,enable);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,enable);
    }
}

