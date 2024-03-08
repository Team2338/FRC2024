package team.gif.robot.commands.autos;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.led.FlashLEDTargetAlign;

public class AutoRotate extends Command {
    double xOffset;
    boolean isComplete;
    private SlewRateLimiter turnLimiter;

    public AutoRotate() {
        super();
//        addRequirements(Robot.swerveDrivetrain); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isComplete = false;
        turnLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double xOffset;
        double rot;

        if (Robot.limelightShooter.hasTarget()) {
            xOffset = Robot.limelightShooter.getXOffset();
            if (xOffset <= -2.0 || xOffset >= 2.0) {
                rot = Robot.driveSwerve.limelightRotateMath(xOffset);
                rot = turnLimiter.calculate(rot) * Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
                Robot.swerveDrivetrain.drive(0,0,rot);
            } else {
                isComplete = true;
                new FlashLEDTargetAlign().schedule();
                Robot.swerveDrivetrain.drive(0,0,0);
            }
        } else {
            // ToDo need to add time based
            isComplete = true;
        }

/*
        // leave in case the above doesn't meet drivers needs
        double pGain = 0.012;
        double ffGain = 0.10;

        if (Robot.limelightShooter.hasTarget()) {
            xOffset = Robot.limelightShooter.getXOffset();
//            System.out.println( "xoffset: " + xOffset + "rot: " + (xOffset>0?-1:1)*(ffGain+pGain*Math.abs(xOffset)));
            if (xOffset <= -2.0 || xOffset >= 2.0) {
                Robot.swerveDrivetrain.drive(0,0,(xOffset>0?-1:1)*(ffGain+pGain*Math.abs(xOffset)));
            } else {
                isComplete = true;
                new FlashLEDTargetAlign().schedule();
                Robot.swerveDrivetrain.drive(0,0,0);
            }
        } else {
            // ToDo need to add time based
            isComplete = true;
        }
 */
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return isComplete;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
