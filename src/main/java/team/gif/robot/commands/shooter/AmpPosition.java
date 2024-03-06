package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class AmpPosition extends Command {

    public AmpPosition() {
        super();
        addRequirements(Robot.shooter,Robot.wrist,Robot.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.wrist.setWristWallPosition();
        Robot.wrist.setTargetPosition(Robot.nextShot.getWristAngle());

        // need to move elevater and rev flywheel (here or in execute)
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // runs until interrupted by AmpShoot (via addRequirements)
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
