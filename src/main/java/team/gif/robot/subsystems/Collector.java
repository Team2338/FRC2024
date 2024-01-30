package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Collector extends SubsystemBase {
    public static TalonSRX collector;
    public Collector(){
        collector = new TalonSRX(RobotMap.COLLECTOR_ID);
        collector.configFactoryDefault();
        collector.setNeutralMode(NeutralMode.Brake);
    }
    public void setCollect(Double Output) {
        collector.set(ControlMode.PercentOutput,Output);
    }
}
