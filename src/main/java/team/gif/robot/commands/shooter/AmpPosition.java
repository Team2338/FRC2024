package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.shootParams;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AmpPosition extends Command {

     boolean finished;

    public AmpPosition() {
        super();
        addRequirements(Robot.wrist,Robot.elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.nextShot = shootParams.AMP;
        finished = false;
        // need to move elevater and rev flywheel (here or in execute)
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.elevator.setTargetPosition(Constants.Elevator.AMP_POS);
        if (Robot.elevator.getPosition() >= Constants.Elevator.AMP_POS) { // defensive code to prevent breaking the wrist
            Robot.wrist.setTargetPosition(Robot.nextShot.getWristAngle());
            finished = true;
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // runs until interrupted by AmpShoot (via addRequirements)
        return finished;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
}
