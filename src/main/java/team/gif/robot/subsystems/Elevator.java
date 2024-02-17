package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Elevator extends SubsystemBase {
    public static CANSparkMax elevator;
    public static SparkPIDController pidElevator;

    public Elevator(){
        elevator = new CANSparkMax(RobotMap.ELEVATOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    }

    public double getPosition(){
        return elevator.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }

    /**
     *  All the config setting for Shooter (controller, pid)
     */
    public void configElevator() {
//        shooterNeo.restoreFactoryDefaults(); // Leave for shooter Neo
//        shooterNeo.setInverted(true); // Leave for shooter Neo
//        shooterNeo.setIdleMode(CANSparkBase.IdleMode.kCoast); // Leave for shooter Neo

        elevator.restoreFactoryDefaults();
        elevator.setInverted(true);
        elevator.setIdleMode(CANSparkBase.IdleMode.kBrake);

//        pidShooter = shooterNeo.getPIDController(); // Leave for shooter Neo
        pidElevator = elevator.getPIDController();

        pidElevator.setP(Constants.Elevator.kP);
        pidElevator.setFF(Constants.Elevator.FF);
        pidElevator.setI(Constants.Elevator.kI);
        pidElevator.setOutputRange(0,1);
    }
}
