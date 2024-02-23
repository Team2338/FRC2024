package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team.gif.robot.commands.drivetrainPbot.Reset0;
import team.gif.robot.commands.drivetrainPbot.Reset180;

public class UI {
    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2024");

//        shuffleboardTab.addDouble("Shooter Voltage", Robot.shooter::getVoltage);

//        shuffleboardTab.addString("Shooter RPM", Robot.shooter::getShooterRPM_Shuffleboard);
        shuffleboardTab.addDouble("Shooter RPM", Robot.shooter::getShooterRPM);

//        SmartDashboard.putData("Reset", new Reset0());
//        SmartDashboard.putData("Reset 180", new Reset180());

        shuffleboardTab.addBoolean("Collector Sensor", Robot.collector::getSensorState);
        shuffleboardTab.addBoolean("Shooter Sensor", Robot.indexer::getSensorState);
        shuffleboardTab.addBoolean("Mid Sensor", Robot.indexer::getStageOneSensorState);

//        shuffleboardTab.addDouble("Shooter Angle", Robot.shooter::get)


        shuffleboardTab.addBoolean("Collector Manual Control", Robot.collector::getCollectorManualControl);
        shuffleboardTab.addBoolean("Indexer Manual Control", Robot.indexer::getIndexerManualFlag);

        shuffleboardTab.addString("Shooter Actual", Robot.shooter::getPosition_Shuffleboard);
        shuffleboardTab.addDouble("Shooter Target", Robot.shooter::getTargetPosition);
        shuffleboardTab.addString("Shooter Degrees", Robot.shooter::getRotationDegrees_Shuffleboard);

        shuffleboardTab.addBoolean("Stage Safe", Robot.diagnostics::getSafeToDriveUnderStage);

        ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("Diagnostics");

        // showing the boolean returned from the getDriveMotorTempCheck
        diagnosticsTab.addBoolean("Swerve Module Temp", Robot.diagnostics::getDriveMotorTempCheck);
        diagnosticsTab.addBoolean("Indexer Temp", Robot.diagnostics::getIndexerMotorTempCheck);
        diagnosticsTab.addBoolean("Shooter Temp", Robot.diagnostics::getShooterMotorTempCheck);
    }
}
