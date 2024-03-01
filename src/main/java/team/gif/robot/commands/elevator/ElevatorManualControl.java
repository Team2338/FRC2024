package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class ElevatorManualControl extends Command {

    private boolean holdNeedFirstPID;
    private double holdPIDPos;

    public ElevatorManualControl() {
        super();
        addRequirements(Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        holdNeedFirstPID = false;
        holdPIDPos = Robot.elevator.getPosition();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double percent = -Robot.oi.aux.getLeftY();

/*
        if (percent > -0.09 && percent < 0.09) { // creates a dead band around the joystick
            Robot.elevator.PIDHold();
        } else {
            if( percent > 0 ) {
                Robot.elevator.move(0.25);
            } else {
                Robot.elevator.move(0.05);
            }
        }
*/
        if (percent > -0.09 && percent < 0.09) {  // creates a dead band around the joystick
            if( holdNeedFirstPID ) {
                holdPIDPos = Robot.elevator.getPosition();
                holdNeedFirstPID = false;
            }
            Robot.elevator.setTargetPosition(holdPIDPos);
            Robot.elevator.PIDHold();
        } else {
            holdNeedFirstPID = true;
            if( percent > 0 ) {
                Robot.elevator.move(0.25);
            } else {
                Robot.elevator.move(0.05);
            }
        }


/*        boolean moveSlow;

        if (Robot.manualControlMode) {
            double percent = -Robot.oi.aux.getHID().getRightY(); // direction of climber

            if (percent > 0.15) {
                    Robot.climber.up(moveSlow);
            } else if (percent < -0.15) {
                    Robot.climber.down(moveSlow);
            } else {
                Robot.climber.hold();
            }
        }
*/
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return !Robot.manualControlMode;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setTargetPosition(Robot.elevator.getPosition());
    }
}
