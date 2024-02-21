package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Elevator extends SubsystemBase {
    public static CANSparkMax elevator;
    public static SparkPIDController pidElevator;

    public Elevator() {
        elevator = new CANSparkMax(RobotMap.ELEVATOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        elevator.restoreFactoryDefaults();
        elevator.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pidElevator = elevator.getPIDController();
        pidElevator.setP(Constants.Shooter.kP);
        pidElevator.setFF(Constants.Shooter.FF);
        pidElevator.setI(Constants.Shooter.kI);
        pidElevator.setOutputRange(0,1);
    }

    public void setElevatorPercent(double percent) {
        elevator.set(percent);
    }

    public void setElevatorPos(double pos) {
        pidElevator.setReference(pos, CANSparkBase.ControlType.kPosition);
    }

    public double getPosition() {
        return elevator.getEncoder().getPosition();
    }

    public String getPosition_Shuffleboard() {
        return String.format("%12.0f", getPosition());
    }

    public void zeroElevator() {
        elevator.getEncoder().setPosition(0);
    }
}
