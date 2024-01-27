package team.gif.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Shooter extends SubsystemBase {
    CANSparkMax shooter = new CANSparkMax(RobotMap.SPARK_ID, CANSparkLowLevel.MotorType.kBrushless);
    public Shooter(){
    }
    public void setShooter(double percent){
        shooter.setVoltage(percent);
    }
}
