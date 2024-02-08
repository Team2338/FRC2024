package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Indexer extends SubsystemBase {
    public static TalonSRX stageOne;
    public static CANSparkMax stageTwo;
    public static SparkPIDController pidControllerStage2;

    public static DigitalInput stageSensor;

    public boolean indexerManualFlag = false;

    public Indexer() {
        stageOne = new TalonSRX(RobotMap.STAGE_ONE_ID);
        stageOne.configFactoryDefault();
        stageOne.setNeutralMode(NeutralMode.Brake);

        stageTwo = new CANSparkMax(RobotMap.STAGE_TWO_ID, CANSparkLowLevel.MotorType.kBrushless);
        stageTwo.restoreFactoryDefaults();
        stageTwo.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pidControllerStage2 = stageTwo.getPIDController();
        pidControllerStage2.setFF(Constants.Indexer.STAGE_SHOOTER_FF);
        pidControllerStage2.setP(Constants.Indexer.STAGE_SHOOTER_kP);

        stageSensor = new DigitalInput(RobotMap.SENSOR_INDEXER_PORT);
    }

    public void setIndexer(double stageOnePercent, double stageTwoPercent) {
        stageOne.set(ControlMode.PercentOutput, stageOnePercent);
        stageTwo.set(stageTwoPercent);
    }

    public void stopIndexer() {
        stageOne.set(ControlMode.PercentOutput,0);
        stageTwo.set(0);
    }

    public boolean getSensorState() {
        return stageSensor.get();
    }

    public boolean getIndexerManualFlag() {
        return indexerManualFlag;
    }
}
