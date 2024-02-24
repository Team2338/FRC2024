package team.gif.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team.gif.lib.autoMode;
import team.gif.lib.delay;
import team.gif.robot.commands.collector.ToggleCollectorDefault;
import team.gif.robot.commands.drivetrain.Reset0;
import team.gif.robot.commands.drivetrain.Reset180;


public class UiSmartDashboard {

    public SendableChooser<delay> delayChooser = new SendableChooser<>();
    public SendableChooser<autoMode> autoModeChooser = new SendableChooser<>();

    public UiSmartDashboard() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("SmartDashboard");

        shuffleboardTab.add("Pigeon Heading", Robot.pigeon.get360Heading());

        shuffleboardTab.add("BotHead", (x) -> {
                    x.setSmartDashboardType("Gyro");
                    x.addDoubleProperty("Value", () -> Robot.pigeon.getCompassHeading(), null);
                })
                .withPosition(5, 0);

        autoModeChooser.addOption("NONE", autoMode.NONE);
        autoModeChooser.addOption("Circle", autoMode.CIRCLE);
        autoModeChooser.addOption("Mobility", autoMode.MOBILITY);
        autoModeChooser.addOption("CTR-C", autoMode.CTR_C);


        shuffleboardTab.add("Auto Select", autoModeChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(7, 1)
                .withSize(2, 1);

        SmartDashboard.putData("Toggle Collector Default", new ToggleCollectorDefault());


        //Auto Delay
        delayChooser.setDefaultOption("0", delay.DELAY_0);
        delayChooser.addOption("1", delay.DELAY_1);
        delayChooser.addOption("2", delay.DELAY_2);
        delayChooser.addOption("3", delay.DELAY_3);
        delayChooser.addOption("4", delay.DELAY_4);
        delayChooser.addOption("5", delay.DELAY_5);
        delayChooser.addOption("6", delay.DELAY_6);
        delayChooser.addOption("7", delay.DELAY_7);
        delayChooser.addOption("8", delay.DELAY_8);
        delayChooser.addOption("9", delay.DELAY_9);
        delayChooser.addOption("10", delay.DELAY_10);
        delayChooser.addOption("11", delay.DELAY_11);
        delayChooser.addOption("12", delay.DELAY_12);
        delayChooser.addOption("13", delay.DELAY_13);
        delayChooser.addOption("14", delay.DELAY_14);
        delayChooser.addOption("15", delay.DELAY_15);

        SmartDashboard.putData("Reset", new Reset0());
        SmartDashboard.putData("Reset 180", new Reset180());
        
        shuffleboardTab.add("Delay", delayChooser)
                .withPosition(7, 0)
                .withSize(1, 1);
    }

    public void updateUI() {
        // Timers
        SmartDashboard.putString("Time", String.format("%.4f", Timer.getFPGATimestamp()));
        SmartDashboard.putString("Timer", String.format("%.2f", Timer.getMatchTime()));
    }
}
