package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class UI {

    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2024");

//        shuffleboardTab.addDouble("Shooter Voltage", Robot.shooter::getVoltage);

        // MK3 debugging code
        shuffleboardTab.addDouble("fr raw encoder", Robot.practiceDrivetrain::fREncoder);
        shuffleboardTab.addDouble("fl raw encoder", Robot.practiceDrivetrain::fLEncoder);
        shuffleboardTab.addDouble("rr raw encoder", Robot.practiceDrivetrain::rREncoder);
        shuffleboardTab.addDouble("rl raw encoder", Robot.practiceDrivetrain::rLEncoder);

        shuffleboardTab.addDouble("fr head", Robot.practiceDrivetrain::fRHead);
        shuffleboardTab.addDouble("fl head", Robot.practiceDrivetrain::fLHead);
        shuffleboardTab.addDouble("rr head", Robot.practiceDrivetrain::rRHead);
        shuffleboardTab.addDouble("rl head", Robot.practiceDrivetrain::rLHead);

        shuffleboardTab.addDouble("fr target", Robot.practiceDrivetrain::fRTarget);
        shuffleboardTab.addDouble("fl target", Robot.practiceDrivetrain::fLTarget);
        shuffleboardTab.addDouble("rr target", Robot.practiceDrivetrain::rRTarget);
        shuffleboardTab.addDouble("rl target", Robot.practiceDrivetrain::rLTarget);

        shuffleboardTab.addDouble("fL ticks", Robot.practiceDrivetrain::fLEncoder);
        shuffleboardTab.addDouble("fr ticks", Robot.practiceDrivetrain::fREncoder);
        shuffleboardTab.addDouble("rL ticks", Robot.practiceDrivetrain::rLEncoder);
        shuffleboardTab.addDouble("rr ticks", Robot.practiceDrivetrain::rREncoder);
    }
}
