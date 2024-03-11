package team.gif.robot.commands.climber;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.ToggleCollectorDefault;
import team.gif.robot.commands.led.FlashLEDTargetAlign;

public class AutoClimb extends Command {
    private SlewRateLimiter strafeLimiter;
    boolean alignComplete;
    double counter;
    boolean driveForwardDone;
    boolean climberRaisedToTop;
    boolean elevatorRaisedToTop;
    boolean climbComplete;
    boolean isFinished;

    public AutoClimb() {
        super();
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        alignComplete = false;
        strafeLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);

        counter = 0;
        driveForwardDone = false;
        climberRaisedToTop = false;
        elevatorRaisedToTop = false;
        climbComplete = false;

        new ToggleCollectorDefault().schedule();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double strafe;
        double xOffset; // limelight x offset

        // auto strafe
        if (Robot.limelightShooter.hasTarget()) {
            xOffset = Robot.limelightShooter.getXOffset();
            if (xOffset <= -1.0 || xOffset >= 1.0) {
                strafe = Robot.driveSwerve.limelightRotateMath(xOffset);
                strafe = strafeLimiter.calculate(strafe) * Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND;
                Robot.swerveDrivetrain.drive(strafe,0,0);
            } else {
                alignComplete = true;
                new FlashLEDTargetAlign().schedule();
                Robot.swerveDrivetrain.drive(0,0,0);
            }
        } else {
            alignComplete = true;
        }

        if (alignComplete) {
            if (counter <= 1.0*50) {
                Robot.swerveDrivetrain.drive(0,0.5,0);
                counter++;
            } else {
                Robot.swerveDrivetrain.drive(0,0,0);
                driveForwardDone = true;
                alignComplete = false;
            }
        }

        if (driveForwardDone) {
            if (Robot.climber.getPosition() < Constants.Climber.LIMIT_MAX) {
                Robot.climber.move(1.0);
            } else {
                Robot.climber.move(0.0);
                climberRaisedToTop = true;
                driveForwardDone = false;
            }
        }

        if (climberRaisedToTop) {
            Robot.elevator.setTargetPosition(Constants.Elevator.LIMIT_MAX);
            climberRaisedToTop = false;
            elevatorRaisedToTop = true;
        }

        if (elevatorRaisedToTop) {
            if (Robot.climber.getPosition() < Constants.Climber.LIMIT_MIN) {
                Robot.climber.move(-0.8);
                if (Robot.climber.getPosition() < Constants.Climber.TRAP_MOVE_ELEVATOR_POS) {
                    Robot.elevator.setTargetPosition(Constants.Elevator.TRAP_POS);
                }
            } else {
                climbComplete = true;
            }
        }

        if (Robot.climber.getPosition() < (Constants.Climber.LIMIT_MIN*.98) && Robot.elevator.getPosition() < (Constants.Elevator.TRAP_POS*.98)) {
            isFinished = true;
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
