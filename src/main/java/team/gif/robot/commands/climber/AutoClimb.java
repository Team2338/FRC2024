package team.gif.robot.commands.climber;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.ToggleCollectorDefault;
import team.gif.robot.commands.led.FlashLEDTargetAlign;
import team.gif.robot.commands.shooter.TrapShoot;

public class AutoClimb extends Command {
    boolean climberLowered;
    boolean elevatorRaisedToTop;
    boolean isFinished;

    int counter;

    public AutoClimb() {
        super();
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevatorRaisedToTop = false;
        climberLowered = false;

        counter = 0;

        new ToggleCollectorDefault().schedule();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        // Raise the elevator
        if (!elevatorRaisedToTop) {
            if (Robot.elevator.getPosition() < Constants.Elevator.TRAP_UP_MIN_POS ) {
                System.out.println(counter++ + " " + Robot.elevator.getPosition());
               Robot.elevator.setTargetPosition(Constants.Elevator.TRAP_UP_POS);
            } else {
                elevatorRaisedToTop = true;
                isFinished = true;
                System.out.println("done");
                System.out.println(counter++ + " " + Robot.elevator.getPosition());
            }
        }
/*
        // lower the climber and elevator
        if (elevatorRaisedToTop && !climberLowered) {
            if (Robot.climber.getPosition() > Constants.Climber.LIMIT_MIN) {
                Robot.climber.move(-0.8);
                if (Robot.climber.getPosition() < Constants.Climber.TRAP_MOVE_ELEVATOR_POS) {
                    Robot.elevator.setTargetPosition(Constants.Elevator.TRAP_POS);
                }
            } else {
                climberLowered = true;
                Robot.climber.move(0);
                Robot.climber.setTargetPosition(Robot.climber.getPosition());
                Robot.climber.setDefaultCommand(new ClimberPIDControl());
            }
        }

        if (elevatorRaisedToTop && !climberLowered) {
            new TrapShoot().schedule();
            isFinished = true;
        }
*/
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // hold the elevator
        // nothing to do - PID already holding elevator

        // hold the climber
        Robot.climber.setTargetPosition(Robot.climber.getPosition());
        Robot.climber.setDefaultCommand(new ClimberPIDControl());
    }
}
