package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class DriveSwerve extends Command {
    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;
    private final SlewRateLimiter turnLimiter;

    public DriveSwerve() {
        this.forwardLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.strafeLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turnLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        addRequirements(Robot.swerveDrivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (Robot.isCompBot) {
            double forwardSign;
            double strafeSign;

            double forward = -Robot.oi.driver.getLeftY(); // need to invert because -Y is away, +Y is pull back
            forward = (Math.abs(forward) > Constants.Joystick.DEADBAND) ? forward : 0.0; //0.00001;

            double strafe = -Robot.oi.driver.getLeftX(); // need to invert because -X is left, +X is right
            strafe = (Math.abs(strafe) > Constants.Joystick.DEADBAND) ? strafe : 0.0;

            double rot = -Robot.oi.driver.getRightX(); // need to invert because left is negative, right is positive
            rot = (Math.abs(rot) > Constants.Joystick.DEADBAND) ? rot : 0.0;

            forwardSign = forward/Math.abs(forward);
            strafeSign = strafe/Math.abs(strafe);
            // Use a parabolic curve (instead if linear) for the joystick to speed ratio
            // This allows for small joystick inputs to use slower speeds
            forward = Math.abs(forward) * forward;
            strafe = Math.abs(strafe) * strafe;

            forward = .5 * Math.sqrt(2 + forward*forward - strafe*strafe + 2*forward*Math.sqrt(2)) -
                    .5 * Math.sqrt(2 + forward*forward - strafe*strafe - 2*forward*Math.sqrt(2));

            strafe = .5 * Math.sqrt(2 - forward*forward + strafe*strafe + 2*strafe*Math.sqrt(2)) -
                    .5 * Math.sqrt(2 - forward*forward + strafe*strafe - 2*strafe*Math.sqrt(2));

            if( Double.isNaN(forward) )
                forward = forwardSign;
            if( Double.isNaN(strafe) )
                strafe = strafeSign;

            //Forward speed, Sideways speed, Rotation Speed
            forward = forwardLimiter.calculate(forward) * Constants.ModuleConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
            strafe = strafeLimiter.calculate(strafe) * Constants.ModuleConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;

            if (Robot.oi.driver.getHID().getRightStickButton()) {
                // use limelight target for rotation
                rot = limelightRotate(forward, strafe);
            } else {
                rot = turnLimiter.calculate(rot) * Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
            }

            // the robot starts facing the driver station so for this year negating y and x
            Robot.swerveDrivetrain.drive(forward, strafe, rot);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    private double limelightRotate(double forward, double strafe) {
        double xOffset;
        double pGain = 0.012;
        double ffGain = 0.10;
        double result = 0;
        double strafeAdjust;
        double kP = 0.0357;

//        System.out.println("fwd: " + forward + " strafe: " + strafe);
        if (Robot.limelightShooter.hasTarget()) {
            xOffset = Robot.limelightShooter.getXOffset();
            System.out.println("xOffset = " + xOffset);

//            if (xOffset <= -2.0 || xOffset >= 2.0) {
//                    result = xOffset * kP;
                    result = turnLimiter.calculate(xOffset * kP) * Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
                    result *= -1.0;
                    //                  result = (xOffset > 0 ? -1 : 1) * result;

//                if (Math.abs(strafe) < 0.3) {
//                    result = (xOffset > 0 ? -1 : 1) * (ffGain + pGain * Math.abs(xOffset));
//                    System.out.println("stationary");
//                }  else {
//                    result = (xOffset > 0 ? -1 : 1) * (ffGain + (9.0 * strafe * strafe) * pGain * Math.abs(xOffset));
//                System.out.println("rot: " + result);
                }
                return (result);
            }
//        }
//        return 0;
//    }
}
