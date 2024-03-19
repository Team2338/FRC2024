package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Map;

import static team.gif.robot.Robot.climber;
import static team.gif.robot.Robot.elevator;

public class UI {
    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2024");

        ShuffleboardLayout safeStageNumbers = shuffleboardTab
                .getLayout("Positions", BuiltInLayouts.kList)
                .withSize(1, 2)
                .withPosition(4,1)
                .withProperties(Map.of("Label position", "LEFT")); // hide labels for commands

        safeStageNumbers.addString("Wrist", Robot.wrist::getWristDegrees_Shuffleboard);
        safeStageNumbers.addString("Elev", Robot.elevator::getPosition_Shuffleboard);
        safeStageNumbers.addString("Climb", Robot.climber::getPosition_Shuffleboard);

        //shuffleboardTab.addBoolean("Stage Safe", Robot.diagnostics::getSafeToDriveUnderStage).withPosition(5,0);

        shuffleboardTab.addBoolean("Shooter Sensor", Robot.indexer::getShooterSensorState).withPosition(6,0);
        shuffleboardTab.addBoolean("Mid Sensor", Robot.indexer::getStageOneSensorState).withPosition(6,1);
        shuffleboardTab.addBoolean("Collector Sensor", Robot.collector::getSensorState).withPosition(6,2);

        shuffleboardTab.addString("Actual", Robot.wrist::getPosition_Shuffleboard);

        if (!Robot.minimalDashboard) {
            /**
             * Additional FRC2024 tab items
             */
            ShuffleboardLayout safeStageIndicators = shuffleboardTab
                    .getLayout("Safe Stage", BuiltInLayouts.kList)
                    .withSize(1, 2)
                    .withPosition(5,1)
                    .withProperties(Map.of("Label position", "LEFT")); // hide labels for commands

            safeStageIndicators.addBoolean("Wrist   ", Robot.diagnostics::getSafeStageWrist);
            safeStageIndicators.addBoolean("Elevator", Robot.diagnostics::getSafeStageElevator);
            safeStageIndicators.addBoolean("Climber ", Robot.diagnostics::getSafeStageClimber);

            shuffleboardTab.addDouble("Shooter RPM", Robot.shooter::getShooterRPM).withPosition(2,1);

            ShuffleboardLayout wristPositions = shuffleboardTab
                    .getLayout("Wrist", BuiltInLayouts.kList)
                    .withSize(1, 2)
                    .withPosition(3,1)
                    .withProperties(Map.of("Label position", "LEFT")); // hide labels for commands

            wristPositions.addString("Actual", Robot.wrist::getPosition_Shuffleboard);
            wristPositions.addString("Target", Robot.wrist::getTargetPosition_Shuffleboard);
            wristPositions.addString("Degrees", Robot.wrist::getWristDegrees_Shuffleboard);

            /**
             * Diagnostics Tab
             */
            ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("Diagnostics");

            // showing the boolean returned from the getDriveMotorTempCheck
            //swerve modules
            diagnosticsTab.addDouble("Swerve fR", Robot.swerveDrivetrain.fR::getDriveTemp).withPosition(0,0);
            diagnosticsTab.addDouble("Swerve fL", Robot.swerveDrivetrain.fL::getDriveTemp).withPosition(1,0);
            diagnosticsTab.addDouble("Swerve rR", Robot.swerveDrivetrain.rR::getDriveTemp).withPosition(2,0);
            diagnosticsTab.addDouble("Swerve rL", Robot.swerveDrivetrain.rL::getDriveTemp).withPosition(3,0);
            diagnosticsTab.addBoolean("Swerve fR Cool", Robot.swerveDrivetrain.fR::isDriveMotorCool).withPosition(0,1);
            diagnosticsTab.addBoolean("Swerve fL Cool", Robot.swerveDrivetrain.fL::isDriveMotorCool).withPosition(1,1);
            diagnosticsTab.addBoolean("Swerve rR Cool", Robot.swerveDrivetrain.rR::isDriveMotorCool).withPosition(2,1);
            diagnosticsTab.addBoolean("Swerve rL Cool", Robot.swerveDrivetrain.rL::isDriveMotorCool).withPosition(3,1);

            //indexer
            diagnosticsTab.addDouble("Indexer One Temp", Robot.indexer::getIndexerOneMotorTemp).withPosition(4,0);
            diagnosticsTab.addDouble("Indexer Two Temp", Robot.indexer::getIndexerTwoMotorTemp).withPosition(5,0);
            diagnosticsTab.addBoolean("Indexer One Cool", Robot.indexer::isIndexerOneCool).withPosition(4,1);
            diagnosticsTab.addBoolean("Indexer Two Cool", Robot.indexer::isIndexerTwoCool).withPosition(5,1);

            //shooter
            diagnosticsTab.addDouble("Shooter Temp", Robot.shooter::getShooterMotorTemp).withPosition(6,0);
            diagnosticsTab.addDouble("Wrist Temp", Robot.wrist::getWristMotorTemp).withPosition(7,0);
            diagnosticsTab.addBoolean("Shooter Cool", Robot.shooter::isShooterCool).withPosition(6,1);
            diagnosticsTab.addBoolean("Wrist Cool", Robot.wrist::isWristCool).withPosition(7,1);

            // elevator and climber
            diagnosticsTab.addDouble("Elevator Temp", Robot.elevator::getMotorTemp).withPosition(3,2);
            diagnosticsTab.addBoolean("Elevator Cool", Robot.elevator::isMotorCool).withPosition(3,3);
            diagnosticsTab.addDouble("Climber Temp", Robot.climber::getMotorTemp).withPosition(4,2);
            diagnosticsTab.addBoolean("Climber Cool", Robot.climber::isMotorCool).withPosition(4,3);

            diagnosticsTab.add("Climber", new InstantCommand(climber::resetPosition).ignoringDisable(true));
            diagnosticsTab.add("Elevator", new InstantCommand(elevator::resetPosition).ignoringDisable(true));

            //collector
            diagnosticsTab.addDouble("Collector Temp", Robot.collector::getMotorTemp).withPosition(8,0);
            diagnosticsTab.addBoolean("Collector Cool", Robot.collector::isStageOneMotorCool).withPosition(8,1);

            // shooterPID
    //        shuffleboardTab.add("kP",0).getEntry();
    //        shuffleboardTab.add("kI",0).getEntry();
    //        shuffleboardTab.add("kD",0).getEntry();

            // used for tuning Shooter PID
    //        createSmartDashboardNumber("FF",0.000155);
    //        createSmartDashboardNumber("kP",0.000300);
    //        createSmartDashboardNumber("kI",0.000000);
        }
    }

    public static double createSmartDashboardNumber(String key, double defValue) {
        double value = SmartDashboard.getNumber(key, defValue);
        SmartDashboard.putNumber(key,defValue);
        return value;
    }
}
