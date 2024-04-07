package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.led.FlashLEDTargetAlign;

public class AutoCollect extends Command {
    boolean isComplete;
    //private SlewRateLimiter turnLimiter;
    private SlewRateLimiter forwardLimiter;
    private SlewRateLimiter strafeLimiter;
    double commandCounter;
    int targetCounter;

    public AutoCollect() {
        super();
//        addRequirements(Robot.swerveDrivetrain); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isComplete = false;
        Robot.killAutoAlign = false;
        //turnLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        strafeLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        forwardLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        commandCounter = 0;
        targetCounter = 0; //Prevent the command from stalling if the robot temporarily looses the target
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double xOffset;
        double yOffset;

        double rot;
        double forward;
        double strafe;

        //If the bot didn't pickup a note in auto it'll skip auto rotate to save time
        if (Robot.runningAutonomousMode && !Robot.sensorMonitor.getShooterSensorState()) {
            isComplete = true;
            return;
        }

        if (Robot.limelightCollector.hasTarget()) {
            xOffset = Robot.limelightCollector.getXOffset();
            yOffset = Robot.limelightCollector.getYOffset();
            targetCounter = 0;

            if (xOffset <= -2.0 || xOffset >= 2.0 && yOffset <= -2.0 || yOffset >= 2.0) {
                if (Robot.collector.getPreSensorState()) {
                    isComplete = true;
                    return;
                }
                //rotation (I'm not sure if we need to rotate)
                //rot = Robot.driveSwerve.limelightRotateMath(xOffset);
                //rot = turnLimiter.calculate(rot) * Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

                //forward
                forward = Robot.driveSwerve.limelightRotateMath(yOffset);
                forward = forwardLimiter.calculate(forward) * Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND;

                //strafe
                strafe = Robot.driveSwerve.limelightRotateMath(xOffset);
                strafe = strafeLimiter.calculate(strafe) * Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND;

                Robot.swerveDrivetrain.drive(forward,strafe,0);
            } else {
                isComplete = true;
                new FlashLEDTargetAlign().schedule();
                Robot.swerveDrivetrain.drive(0,0,0);
            }
        } else {
            targetCounter++;
        }

        if (Robot.killAutoAlign || commandCounter++ >= 2.0*50 || targetCounter > 5) {
            isComplete = true;
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return isComplete;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrivetrain.drive(0,0,0);
    }
}
