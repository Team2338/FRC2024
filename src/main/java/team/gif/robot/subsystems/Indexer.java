package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Indexer extends SubsystemBase {
//    public static TalonSRX stageOne;
    public static CANSparkMax stageOneMotor;
    public static CANSparkMax stageTwoMotor;
    public static SparkPIDController pidControllerStage2;

    private boolean isIndexing;
    private boolean notePassedCollector;

    public Indexer() {
        stageOneMotor = new CANSparkMax(RobotMap.STAGE_ONE_ID, CANSparkLowLevel.MotorType.kBrushless);
        stageOneMotor.restoreFactoryDefaults();
        stageOneMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        stageOneMotor.enableVoltageCompensation(12);

        stageTwoMotor = new CANSparkMax(RobotMap.STAGE_TWO_ID, CANSparkLowLevel.MotorType.kBrushless);
        stageTwoMotor.restoreFactoryDefaults();
        stageTwoMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        stageTwoMotor.enableVoltageCompensation(12);
        stageTwoMotor.setInverted(true);

        pidControllerStage2 = stageTwoMotor.getPIDController();
        pidControllerStage2.setFF(Constants.Indexer.INDEXER_TWO_FF);
        pidControllerStage2.setP(Constants.Indexer.INDEXER_TWO_kP);

        notePassedCollector = true;
        isIndexing = false;
    }

    public void setIndexer(double stageOnePercent, double stageTwoPercent) {
        stageOneMotor.set(stageOnePercent); // 2024 bot
        stageTwoMotor.set(stageTwoPercent);
    }

    public void stopIndexerHard() {
        stageTwoMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        setIndexer(0,0);
    }

    public void stopIndexerCoast() {
        stageTwoMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        setIndexer(0,0);
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

    public double getIndexerOneMotorTemp() {
        return stageOneMotor.getMotorTemperature();
    }

    public double getIndexerTwoMotorTemp() {
        return stageTwoMotor.getMotorTemperature();
    }

    public boolean isIndexerOneCool() {
        return getIndexerOneMotorTemp() < Constants.MotorTemps.INDEXER_MOTOR_TEMP;
    }

    public boolean isIndexerTwoCool() {
        return getIndexerTwoMotorTemp() < Constants.MotorTemps.INDEXER_MOTOR_TEMP;
    }
}
