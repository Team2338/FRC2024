package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import team.gif.lib.autoMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team.gif.robot.commands.drivetrainPbot.Reset0;
import team.gif.robot.commands.drivetrainPbot.Reset180;

public class UI {
    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2024");

//        shuffleboardTab.addDouble("Shooter Voltage", Robot.shooter::getVoltage);

        shuffleboardTab.addString("Shooter RPM", Robot.shooter::getShooterRPM_Shuffleboard);

        SmartDashboard.putData("Reset", new Reset0());
        SmartDashboard.putData("Reset 180", new Reset180());

        shuffleboardTab.addBoolean("Collector Sensor", Robot.collector::getSensorState);
        shuffleboardTab.addBoolean("Indexer Sensor", Robot.indexer::getSensorState);

//        shuffleboardTab.addDouble("Shooter Angle", Robot.shooter::get)

        shuffleboardTab.addBoolean("Collector Manual Control", Robot.collector::getCollectorManualControl);
        shuffleboardTab.addBoolean("Indexer Manual Control", Robot.indexer::getIndexerManualFlag);

        shuffleboardTab.addString("Shooter Actual", Robot.shooter::getPosition_Shuffleboard);
        shuffleboardTab.addDouble("Shooter Target", Robot.shooter::getTargetPosition);
        shuffleboardTab.addString("Shooter Degrees", Robot.shooter::getRotationDegrees_Shuffleboard);

        shuffleboardTab.addString("Elevator Postion", Robot.elevator::getPosition_Shuffleboard);
    }
}
