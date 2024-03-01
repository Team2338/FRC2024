package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static team.gif.robot.Robot.climber;
import static team.gif.robot.Robot.elevator;

public class UI {
    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC2024");

//        shuffleboardTab.addDouble("Shooter Voltage", Robot.shooter::getVoltage);

//        shuffleboardTab.addString("Shooter RPM", Robot.shooter::getShooterRPM_Shuffleboard);
        shuffleboardTab.addDouble("Shooter RPM", Robot.shooter::getShooterRPM);

//        SmartDashboard.putData("Reset", new Reset0());
//        SmartDashboard.putData("Reset 180", new Reset180());

        shuffleboardTab.addBoolean("Collector Sensor", Robot.collector::getSensorState);
        shuffleboardTab.addBoolean("Shooter Sensor", Robot.indexer::getShooterSensorState);
        shuffleboardTab.addBoolean("Mid Sensor", Robot.indexer::getStageOneSensorState);

//        shuffleboardTab.addDouble("Shooter Angle", Robot.shooter::get)


        shuffleboardTab.addBoolean("Collector Manual Control", Robot.collector::getCollectorManualControl);
        shuffleboardTab.addBoolean("Indexer Manual Control", Robot.indexer::getIndexerManualFlag);

        shuffleboardTab.addString("Wrist Actual", Robot.wrist::getPosition_Shuffleboard);
        shuffleboardTab.addDouble("Wrist Target", Robot.wrist::getTargetPosition);
        shuffleboardTab.addString("Wrist Degrees", Robot.wrist::getWristDegrees_Shuffleboard);

        shuffleboardTab.addBoolean("Stage Safe", Robot.diagnostics::getSafeToDriveUnderStage);

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

        diagnosticsTab.add("Climber", new InstantCommand(climber::resetPosition).ignoringDisable(true));

        // shooterPID
        shuffleboardTab.add("kP",0).getEntry();
        shuffleboardTab.add("kI",0).getEntry();
        shuffleboardTab.add("kD",0).getEntry();

        //collector
        diagnosticsTab.addDouble("Collector Temp", Robot.collector::getMotorTemp).withPosition(8,0);
        diagnosticsTab.addBoolean("Collector Cool", Robot.collector::isStageOneMotorCool).withPosition(8,1);

        // used for tuning Shooter PID
//        createSmartDashboardNumber("FF",0.000155);
//        createSmartDashboardNumber("kP",0.000300);
//        createSmartDashboardNumber("kI",0.000000);
    }

    public static double createSmartDashboardNumber(String key, double defValue) {
        double value = SmartDashboard.getNumber(key, defValue);
        SmartDashboard.putNumber(key,defValue);
        return value;
    }
}
