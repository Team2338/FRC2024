package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class UI {

    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2024");
        shuffleboardTab.addDouble("fr", Robot.practiceDrivetrain::fREncoder);
        shuffleboardTab.addDouble("fl", Robot.practiceDrivetrain::fLEncoder);
        shuffleboardTab.addDouble("rr", Robot.practiceDrivetrain::rREncoder);
        shuffleboardTab.addDouble("rl", Robot.practiceDrivetrain::rLEncoder);

    }
}
