package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
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

    public static DigitalInput midSensor;
    public static DigitalInput shooterSensor;

    public boolean indexerManualFlag = false;
    private boolean isIndexing;
    private boolean notePassedCollector;

    public Indexer() {
        // 2023 bot
//        stageOne = new TalonSRX(RobotMap.STAGE_ONE_ID);
//        stageOne.configFactoryDefault();
//        stageOne.setNeutralMode(NeutralMode.Brake);

        // 2024 bot
        stageOne = new CANSparkMax(RobotMap.STAGE_ONE_ID, CANSparkLowLevel.MotorType.kBrushless);
        stageOne.restoreFactoryDefaults();
        stageOne.setIdleMode(CANSparkBase.IdleMode.kBrake);

        stageTwo = new CANSparkMax(RobotMap.STAGE_TWO_ID, CANSparkLowLevel.MotorType.kBrushless);
        stageTwo.restoreFactoryDefaults();
        stageTwo.setIdleMode(CANSparkBase.IdleMode.kBrake);
        stageTwo.setInverted(true);

        pidControllerStage2 = stageTwo.getPIDController();
        pidControllerStage2.setFF(Constants.Indexer.INDEXER_TWO_FF);
        pidControllerStage2.setP(Constants.Indexer.INDEXER_TWO_kP);

        midSensor = new DigitalInput(RobotMap.MIDDLE_SENSOR_PORT);
        shooterSensor = new DigitalInput(RobotMap.SHOOTER_SENSOR_PORT);
        notePassedCollector = true;
        isIndexing = false;
    }

    public void setIndexer(double stageOnePercent, double stageTwoPercent) {
//        stageOne.set(ControlMode.PercentOutput, stageOnePercent); // 2023 bot
        stageOne.set(stageOnePercent); // 2024 bot
        stageTwo.set(stageTwoPercent);
    }

    public void stopIndexer() {
        setIndexer(0,0);
    }

    public boolean getSensorState() {
        return shooterSensor.get();
    }

    public boolean getStageOneSensorState() {
        return midSensor.get();
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
