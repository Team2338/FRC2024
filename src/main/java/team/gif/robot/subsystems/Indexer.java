package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Indexer extends SubsystemBase {
    public TalonSRX indexer_stage1 = new TalonSRX(RobotMap.STAGE_1);
    public TalonSRX indexer_stage2 = new TalonSRX(RobotMap.STAGE_2);
    public Indexer(){}
    public void Index(Double Output,Double OutputA){
        indexer_stage1.set(TalonSRXControlMode.PercentOutput,Output);
        indexer_stage2.set(TalonSRXControlMode.PercentOutput,OutputA);
    }
}
