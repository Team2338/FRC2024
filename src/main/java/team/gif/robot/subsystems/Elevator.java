package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Elevator extends SubsystemBase {
    public static CANSparkMax motor;
    public static SparkPIDController pidController;
    public static double targetPosition;

    public Elevator(){
        motor = new CANSparkMax(RobotMap.ELEVATOR_ID, CANSparkLowLevel.MotorType.kBrushless);
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
     * Used to move elevator in manual mode, using joystick % is input
     * @param percent
     */
    public void move(double percent) {
        // soft limits will keep the robot arm in allowable range
        motor.set(percent);
    }

    public void PIDHold() {
//        move(0.03); // used for percent hold
        motor.getPIDController().setReference(targetPosition,CANSparkBase.ControlType.kPosition);
    }

    public String getPosition_Shuffleboard() {
        return String.format("%3.3f", getPosition());
    }

    public double getMotorTemp() {
        return motor.getMotorTemperature();
    }

    public boolean isMotorCool() {
        return getMotorTemp() <= Constants.MotorTemps.ELEVATOR_MOTOR_TEMP;
    }

    /**
     *  All the config setting for Elevator (controller, pid)
     */
    public void config() {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.enableVoltageCompensation(12);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,(float) Constants.Elevator.LIMIT_MAX);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,(float) Constants.Elevator.LIMIT_MIN);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,true);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,true);

        motor.setInverted(false);

        pidController = motor.getPIDController();
        pidController.setFF(Constants.Elevator.FF);
        pidController.setP(Constants.Elevator.kP);
        pidController.setI(Constants.Elevator.kI);
        pidController.setD(Constants.Elevator.kD);
        pidController.setOutputRange(-0.12,1);
    }

    public void enableSoftLimits(boolean enable) {
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,enable);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,enable);
    }
}
