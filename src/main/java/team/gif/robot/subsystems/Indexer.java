package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Indexer extends SubsystemBase {
    public static TalonSRX stageOne;
    public static TalonSRX stageTwo;

    public Indexer() {
        stageOne = new TalonSRX(RobotMap.STAGE_ONE);
        stageOne.configFactoryDefault();
        stageOne.setNeutralMode(NeutralMode.Brake);

        stageTwo = new TalonSRX(RobotMap.STAGE_TWO);
        stageTwo.configFactoryDefault();
        stageTwo.setNeutralMode(NeutralMode.Brake);
    }

    public void setIndexer(double stageOnePercent, double stageTwoPercent) {
        stageOne.set(ControlMode.PercentOutput, stageOnePercent);
        stageTwo.set(ControlMode.PercentOutput, stageTwoPercent);
    }
}
