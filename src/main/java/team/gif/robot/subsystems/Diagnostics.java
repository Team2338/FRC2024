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

    public boolean getClimberMotorTempHot() {
        return (Robot.climber.getMotorTemp() >= Constants.MotorTemps.CLIMBER_MOTOR_TEMP);
    }

    public boolean getElevatorMotorTempHot() {
        return (Robot.elevator.getMotorTemp() >= Constants.MotorTemps.ELEVATOR_MOTOR_TEMP);
    }

    public boolean getAnyMotorTempHot() {
        return  getShooterMotorTempHot() ||
                getIndexerMotorTempHot() ||
                getDriveMotorTempHot()   ||
                getClimberMotorTempHot() ||
                getElevatorMotorTempHot();
    }

    public boolean getSafeToDriveUnderStage() {
        boolean result;

        // all the conditions to indicate the robot can drive under the stage safely
        result = getSafeStageWrist() &&
                 getSafeStageClimber() &&
                 getSafeStageElevator();

        return result;
    }

    public boolean getSafeStageWrist() {
        return Robot.wrist.getPosition() <= Robot.wrist.degreesToAbsolute(Constants.Wrist.SAFE_STAGE_DEGREES);
    }

    public boolean getSafeStageClimber() {
        return Robot.climber.getPosition() <= Constants.Climber.SAFE_STAGE_POS;
    }

    public boolean getSafeStageElevator() {
        return Robot.elevator.getPosition() <= Constants.Elevator.SAFE_STAGE_POS;
    }
    /**
     *  General purpose method to let other subsystems know if the robot has a note
     * @return true if the robot has a note, false of not
     */
    public boolean getRobotHasNote() {
        return Robot.collector.getSensorState() || Robot.indexer.getStageOneSensorState() || Robot.indexer.getShooterSensorState();
    }

    /**
     * Uses the "box" the limelight draws around April Tags to
     * determine if the cross-hair is within the box
     *
     * @return true if the cross-hair is within the box
     */
    public boolean getTargetAligned() {
        // update the status of the target alignment
        double xOffset = Robot.limelightShooter.getXOffset();
        double targetWidthPixels = Robot.limelightShooter.getLongLength();
        double pixelsPerDegree = 320/29.8; // 320 horizontal pixels equivalent to 29.8 degrees
        double targetWidthDegrees = targetWidthPixels / pixelsPerDegree / 2; // divide by 2 for half of length

        return Robot.limelightShooter.hasTarget() && Math.abs(xOffset) <= targetWidthDegrees;
    }
}
