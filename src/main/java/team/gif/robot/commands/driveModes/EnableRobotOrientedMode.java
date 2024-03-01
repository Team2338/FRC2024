package team.gif.robot.commands.driveModes;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.drivePace;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.SwerveDrivetrain;

public class EnableRobotOrientedMode extends Command {
    public EnableRobotOrientedMode() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.swerveDrivetrain.setDrivePace(drivePace.COAST_RR);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrivetrain.setDrivePace(drivePace.COAST_FR);
    }
}
