package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorManualControl extends Command {
    public CollectorManualControl() {
        super();
        addRequirements(Robot.collector); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (!Robot.sensors.shooter() &&
                Robot.wrist.getPosition() < Constants.Wrist.MIN_COLLECT) {
                //Robot.elevator.getPosition() < Constants.Elevator.MIN_COLLECT) {
            Robot.collector.collect();
        } else {
            Robot.collector.eject();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if (Robot.sensors.indexer()) { // hack for the note moving too far into the indexer when the button is being held
            return true;
        }
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
