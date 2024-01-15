package team.gif.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class NEOTele extends SubsystemBase {
    CANSparkMax sparky = new CANSparkMax(RobotMap.SPARK_MAX_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public NEOTele(){
    }
    public void setSparky(double percent){
        sparky.setVoltage(percent);
    }

}

