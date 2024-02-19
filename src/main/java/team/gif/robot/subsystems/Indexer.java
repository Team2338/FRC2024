package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Indexer extends SubsystemBase {
//    public static TalonSRX stageOne;
    public static CANSparkMax stageOne;
    public static CANSparkMax stageTwo;
    public static SparkPIDController pidControllerStage2;

    public static DigitalInput stageSensor;

    public boolean indexerManualFlag = false;
    private boolean isIndexing;
    private boolean notePassedCollector;

    public Indexer() {
//        stageOne = new TalonSRX(RobotMap.STAGE_ONE_ID);
//        stageOne.configFactoryDefault();
//        stageOne.setNeutralMode(NeutralMode.Brake);

        stageOne = new CANSparkMax(RobotMap.STAGE_ONE_ID, CANSparkLowLevel.MotorType.kBrushless);
        stageOne.restoreFactoryDefaults();
        stageOne.setIdleMode(CANSparkBase.IdleMode.kBrake);

        stageTwo = new CANSparkMax(RobotMap.STAGE_TWO_ID, CANSparkLowLevel.MotorType.kBrushless);
        stageTwo.restoreFactoryDefaults();
        stageTwo.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pidControllerStage2 = stageTwo.getPIDController();
        pidControllerStage2.setFF(Constants.Indexer.STAGE_SHOOTER_FF);
        pidControllerStage2.setP(Constants.Indexer.STAGE_SHOOTER_kP);

        stageSensor = new DigitalInput(RobotMap.SENSOR_INDEXER_PORT);
        notePassedCollector = true;
        isIndexing = false;
    }

    public void setIndexer(double stageOnePercent, double stageTwoPercent) {
        stageOne.set(stageOnePercent);
        stageTwo.set(stageTwoPercent);
    }

    public void stopIndexer() {
        stageOne.set(0);
        stageTwo.set(0);
    }

    public boolean getSensorState() {
        return stageSensor.get();
    }

    public boolean getIndexerManualFlag() {
        return indexerManualFlag;
    }

    /**
     * Sets if the bot is currently indexing a note.
     * There is a gap in the path where the note is not detected by
     * either the collector sensor or the indexing sensor. This keeps
     * track of the state
     *
     * @param state true if the bot is indexing a note, false if not
     */
    public void setIndexing(boolean state) {
        isIndexing = state;
    }

    /**
     * Indicates if the bot is currently indexing a note.
     * There is a gap in the path where the note is not detected by
     * either the collector sensor or the indexing sensor.
     *
     * @return true if the bot is indexing a note, false if not
     */
    public boolean isIndexing() {
        return isIndexing;
    }

    /**
     * Sets if the note has gone passed the
     * collector sensor after entering the bot.
     * This is only valid during indexing.
     *
     * @param state true if passed, false if not
     */
    public void setNotePassedCollector(boolean state) {
        notePassedCollector = state;
    }

    /**
     * Indicates if the note has entered the bot and gone
     * passed the collector sensor. This is only valid
     * during indexing.
     *
     * @return true if passed, false if not
     */
    public boolean getNotePassedCollector() {
        return notePassedCollector;
    }
}
