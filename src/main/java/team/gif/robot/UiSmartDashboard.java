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

        if (Robot.fullDashboard) {
            shuffleboardTab.add("Pigeon Heading", Robot.pigeon.get360Heading());

            shuffleboardTab.add("BotHead", (x) -> {
                        x.setSmartDashboardType("Gyro");
                        x.addDoubleProperty("Value", () -> Robot.pigeon.getCompassHeading(), null);
                    })
                    .withPosition(5, 0);
            SmartDashboard.putData("Reset", new Reset0());
//            SmartDashboard.putData("Reset 180", new Reset180());
        }
        autoModeChooser.addOption("NONE", autoMode.NONE);
//        autoModeChooser.addOption("Circle", autoMode.CIRCLE);
        autoModeChooser.setDefaultOption("Mobility", autoMode.MOBILITY);
//        autoModeChooser.addOption("2CTR+C", autoMode.TWO_CTR_C);
//        autoModeChooser.addOption("2SRC+S", autoMode.TWO_SRC_S);
//        autoModeChooser.addOption("2SRC+8", autoMode.TWO_SRC_8);
//        autoModeChooser.addOption("2SRC+7", autoMode.TWO_SRC_7); // TODO: testing need, I'm not sure about this
        autoModeChooser.addOption("2W+8+7", autoMode.TWO_W_8_7);

        autoModeChooser.addOption("3W+8+7", autoMode.THREE_W_8_7);
        autoModeChooser.addOption("3W+7+8", autoMode.THREE_W_7_8);
        autoModeChooser.addOption("3W+7+6", autoMode.THREE_W_7_6);
        autoModeChooser.addOption("3W+6+7", autoMode.THREE_W_6_7);

        autoModeChooser.addOption("4CTR+C+S+A-4", autoMode.FOUR_CTR_C_S_A_4);

        autoModeChooser.addOption("5CTR+C+S+A+4", autoMode.FIVE_CTR_C_S_A_4);
        autoModeChooser.addOption("5CTR+C+S+A+5", autoMode.FIVE_CTR_C_S_A_5);
        autoModeChooser.addOption("5CTR+C+S+A+6", autoMode.FIVE_CTR_C_S_A_6);

        autoModeChooser.addOption("Shooter Test", autoMode.SHOOTER_TEST);
//        autoModeChooser.addOption("2SCSplit+6", autoMode.TWO_SCSPLIT_SIX);
//        autoModeChooser.addOption("4AMP+A+C+S", autoMode.FOUR_AMP_A_C_S);
//        autoModeChooser.addOption("Line Test", autoMode.LINE_TEST);

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


        shuffleboardTab.add("Delay", delayChooser)
                .withPosition(7, 0)
                .withSize(1, 1);

        shuffleboardTab.addBoolean("Motor Temp", Robot.diagnostics::getAnyMotorTempHot);

        shuffleboardTab.addBoolean("Stage Safe", Robot.diagnostics::getSafeToDriveUnderStage);

        shuffleboardTab.addBoolean("Ready", Robot.shooter::getShooterAtMinRPM);

        shuffleboardTab.addBoolean("Aligned", Robot.diagnostics::getTargetAligned);

        shuffleboardTab.addBoolean("Manual", Robot::getManualControlMode);
    }

    public void updateUI() {
        if(Robot.fullDashboard) {
            // Timers
            SmartDashboard.putString("Time", String.format("%.4f", Timer.getFPGATimestamp()));
            SmartDashboard.putString("Timer", String.format("%.2f", Timer.getMatchTime()));
        }
    }
}
