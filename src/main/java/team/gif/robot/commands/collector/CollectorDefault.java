package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorDefault extends Command {
    double counter;
    boolean isCollecting;

    public CollectorDefault() {
        super();
        addRequirements(Robot.collector);
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        isCollecting = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.limelightCollector.hasTarget()) {
            Robot.collector.collect();
            isCollecting = true;
            counter = 0;
        } else {
            if (isCollecting) {
                counter++;
                if (counter < (1*50)) {
                    Robot.collector.collect();
                    Robot.collector.collect();
                } else {
                    isCollecting = false;
                }
            } else {
                Robot.collector.eject();
            }
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.collector.stop();
    }
}
