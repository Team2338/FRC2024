package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class RevFlyWheels extends Command {

    public int commandCounter;

    public RevFlyWheels() {
        super();
        addRequirements(Robot.shooter); // uncomment
    }

    public RevFlyWheels(boolean isAuto) {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        commandCounter = 0;
        Robot.shooter.resetKI();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.sensorMonitor.getShooterSensorState()) {
            if (true) { // ToDo need to be able to use direct buttons
                Robot.wrist.setWristAuto();
            } else {
                Robot.wrist.setTargetPosition(Robot.nextShot.getWristAngle());
            }
        }

        Robot.shooter.setupAndRev();

        if (Robot.shooter.getShooterRPM() >= Robot.nextShot.getMinimumRPM() && !Robot.runningAutonomousMode) {
            Robot.oi.setRumble(true);
        } else {
            Robot.oi.setRumble(false);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.runningAutonomousMode && (commandCounter++ > (1.0 * 50));
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // don't want to set shooter RPM to 0 because the shoot command is taking over
        // instead, use onFalse in OI and call shooter.stop()
        Robot.oi.setRumble(false);
    }
}
