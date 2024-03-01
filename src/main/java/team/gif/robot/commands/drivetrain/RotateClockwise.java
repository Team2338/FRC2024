package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class RotateClockwise extends Command {

    double counter;

    public RotateClockwise() {
        addRequirements(Robot.swerveDrivetrain); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.swerveDrivetrain.drive(0,0,0.1);
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (counter > .2 * 50) {
            Robot.swerveDrivetrain.drive(0,0,0.1);
        }
        counter++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrivetrain.drive(0.0, 0.0, 0.0);
    }
}
