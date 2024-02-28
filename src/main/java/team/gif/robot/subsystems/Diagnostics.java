package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class Diagnostics extends SubsystemBase {

    public Diagnostics(){}

    /**
     * getting the temp for the swerve module and checking to see if its too hot
     * @return isTooHot is a boolean
     */
    public boolean getDriveMotorTempHot() {
        return(Robot.swerveDrivetrain.fL.getDriveTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP ||
                Robot.swerveDrivetrain.fR.getDriveTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP ||
                Robot.swerveDrivetrain.rL.getDriveTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP ||
                Robot.swerveDrivetrain.rR.getDriveTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP);
    }

    /**
     * getting the temp of the indexer motor temp (indexer one and two)
     * @return Returns true or false
     */
    public boolean getIndexerMotorTempHot() {
        return(Robot.indexer.getIndexerOneMotorTemp() >= Constants.MotorTemps.INDEXER_MOTOR_TEMP ||
            Robot.indexer.getIndexerTwoMotorTemp() >= Constants.MotorTemps.INDEXER_MOTOR_TEMP);
    }

    /**
     * getting the temp of the shooter motor temp
     * @return returns true of false
     */
    public boolean getShooterMotorTempHot() {
        return (Robot.shooter.getShooterMotorTemp() >= Constants.MotorTemps.SHOOTER_MOTOR_TEMP);
    }

    public boolean getAnyMotorTempHot() {
        return getShooterMotorTempHot() || getIndexerMotorTempHot() || getDriveMotorTempHot();
    }

    public boolean getSafeToDriveUnderStage() {
        return Robot.wrist.getPosition() <= Robot.wrist.degreesToAbsolute(50);//(Robot.climber.getPosition() < 100 && Robot.elevator.getPosition() < 100 && Robot.shooter.getPosition() < 100);
    }
}
