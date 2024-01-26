package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import team.gif.lib.autoMode;

public class UI {
    public UI() {
        ShuffleboardTab tab = Shuffleboard.getTab("FRC2024");
        tab.addDouble("Odom X", Robot.swerveDrivetrain::getPoseX);
        tab.addDouble("Odom Y", Robot.swerveDrivetrain::getPoseY);
    }
}
