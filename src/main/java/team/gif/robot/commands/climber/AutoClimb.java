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

    int counter;

    public AutoClimb() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevatorRaisedToTop = true;
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
                System.out.println("Elevator up: " + counter++ + " " + Robot.elevator.getPosition());
               Robot.elevator.setTargetPosition(Constants.Elevator.TRAP_UP_POS);
            } else {
                elevatorRaisedToTop = true;
                System.out.println("Elevator up: done");
                System.out.println("Elevator up: " + counter + " " + Robot.elevator.getPosition());
            }
        }

        // lower the climber and elevator
        if (elevatorRaisedToTop && !climberLowered) {
            if (Robot.climber.getPosition() > Constants.Climber.TRAP_POS) {
//                System.out.println("Climber down: " + counter++ + " " + Robot.climber.getPosition());
                Robot.climber.move(Constants.Climber.CLIMBER_DOWN_SPEED); //-0.8 for climbing
                if (Robot.climber.getPosition() < Constants.Climber.TRAP_MOVE_ELEVATOR_POS) {
                    Robot.elevator.setTargetPosition(Constants.Elevator.TRAP_POS);
                }
            } else {
                climberLowered = true;
//                System.out.println("Climber Down: done");
//                System.out.println("CLimber down: " + counter + " " + Robot.climber.getPosition());
            }
        }

        if (elevatorRaisedToTop && climberLowered) {
            new TrapShoot().schedule();
            System.out.println("We are done!");
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return elevatorRaisedToTop && climberLowered;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // hold the elevator
        // nothing to do - PID already holding elevator

        // hold the climber
        Robot.climber.move(0);
        Robot.climber.setTargetPosition(Robot.climber.getPosition());
        Robot.climber.setDefaultCommand(new ClimberPIDControl());
        System.out.println("end trap sequence ");
    }
}
