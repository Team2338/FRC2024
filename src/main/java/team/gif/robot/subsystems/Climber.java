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
    public static SparkPIDController pidClimber;

    public Climber(){
        climber = new CANSparkMax(RobotMap.CLIMBER_ID, CANSparkLowLevel.MotorType.kBrushless);
    }

    public double getPosition(){
        return climber.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }

    /**
     *  All the config setting for Shooter (controller, pid)
     */
    public void configElevator() {
//        shooterNeo.restoreFactoryDefaults(); // Leave for shooter Neo
//        shooterNeo.setInverted(true); // Leave for shooter Neo
//        shooterNeo.setIdleMode(CANSparkBase.IdleMode.kCoast); // Leave for shooter Neo

        climber.restoreFactoryDefaults();
        climber.setInverted(true);
        climber.setIdleMode(CANSparkBase.IdleMode.kBrake);

//        pidShooter = shooterNeo.getPIDController(); // Leave for shooter Neo
        pidClimber = climber.getPIDController();

        pidClimber.setP(Constants.Climber.kP);
        pidClimber.setFF(Constants.Climber.FF);
        pidClimber.setI(Constants.Climber.kI);
        pidClimber.setOutputRange(0,1);
    }
}
