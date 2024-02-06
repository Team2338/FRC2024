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

        shuffleboardTab.addString("Shooter RPM", Robot.shooter::getShooterRPM_Shuffleboard);

        SmartDashboard.putData("Reset", new Reset0());
        SmartDashboard.putData("Reset 180", new Reset180());

        shuffleboardTab.addBoolean("Collector Sensor", Robot.collector::getSensorState);
        shuffleboardTab.addBoolean("Indexer Sensor", Robot.indexer::getSensorState);

        shuffleboardTab.addBoolean("Manual Control", Robot.collector::getCollectorManualControl);

        shuffleboardTab.addDouble("Shooter Angle", Robot.shooter::getPosition);
    }
}
