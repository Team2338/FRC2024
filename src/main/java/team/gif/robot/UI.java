package team.gif.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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
        diagnosticsTab.addDouble("Shooter Rotation Temp", Robot.shooter::getShooterRotationMotorTemp).withPosition(7,0);
        diagnosticsTab.addBoolean("Shooter Cool", Robot.shooter::isShooterCool).withPosition(6,1);
        diagnosticsTab.addBoolean("Shooter Rotation Cool", Robot.shooter::isShooterRotationCool).withPosition(7,1);

        //collector
        diagnosticsTab.addDouble("Collector Temp", Robot.collector::getMotorTemp).withPosition(8,0);
        diagnosticsTab.addBoolean("Collector Cool", Robot.collector::isStageOneMotorCool).withPosition(8,1);
    }
}
