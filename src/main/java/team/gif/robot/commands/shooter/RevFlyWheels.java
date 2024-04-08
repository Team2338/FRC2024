package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class RevFlyWheels extends Command {

//    public int commandCounter;

    /**
     * Revs the flywheel according to the next target <br>
     * In teleop:
     *    Sets the wrist target according to next target (or auto) <br>
     *    Wrist will move due to wrist default command using PID <br>
     *    Rumble when shooter reaches minimum RPM <br>
     * In Auto: <br>
     *    Does not move wrist <br>
     *    Does not rumble (to avoid controller from falling off driver station <br>
     *    Can only be used in a parallel race where the other command will end, otherwise will never end
     */
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
//        commandCounter = 0;
        Robot.shooter.resetKI();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // In teleop, move the wrist according to LL distance or next shot
        // In autonomous, do not move wrist and just rev flywheel
        if (!Robot.runningAutonomousMode) {
            if (Robot.sensors.shooter()) {
                if (Robot.wrist.isAutoAngleEnabled()) {
                    Robot.wrist.setWristAuto();
                } else {
                    Robot.wrist.setTargetPosition(Robot.nextShot.getWristAngle());
                }
            }
        }

        Robot.shooter.configMotorControllerAndRev();

        // rumble when the shooter gets to the minimum RPM
        // (do not rumble during auto to prevent controller from falling off ledge)
        if (Robot.shooter.getShooterRPM() >= Robot.nextShot.getMinimumRPM() && !Robot.runningAutonomousMode) {
            Robot.oi.setRumble(true);
        } else {
            Robot.oi.setRumble(false);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false; // Robot.runningAutonomousMode && (commandCounter++ > (1.0 * 50));
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // don't want to set shooter RPM to 0 because the shoot command is taking over
        // instead, use onFalse in OI and call shooter.stop()
        Robot.oi.setRumble(false);
        System.out.println("RevFlyWheel ended");
    }
}
