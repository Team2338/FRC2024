package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.indexer.IndexerDefault;

public class Shoot extends Command {
    boolean isFiring;
    double fireCounter;
    double commandCounter;
    boolean indexingRequired;

    /**
     * Creates a new Shoot command. Pass in true in for auto mode.
     */
    public Shoot() {
        super();
        addRequirements(Robot.indexer,Robot.shooter);
    }

    public Shoot(boolean isAuto) {
        //Pathplanner takes control of the indexer for the entire path
        //so we remove it here so that we can have the indexer default for
        //the rest of the path
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFiring = false;
        fireCounter = 0;
        commandCounter = 0;
        indexingRequired = false;

        Robot.indexer.stopIndexerCoast();
        //We need to remove the default command if we are in autonomous mode
        //because the default command will fight with this command for control
        //of the indexer
        if (Robot.runningAutonomousMode) {
            Robot.indexer.removeDefaultCommand();
            Robot.indexer.getCurrentCommand().cancel();
        }

        // load an initial angle into the next shot
        if (Robot.wrist.isAutoAngleEnabled()) {
            Robot.wrist.setNextShotAuto();
        }
        System.out.println("AutoShot? " + Robot.wrist.isAutoAngleEnabled() + " nextShot " + Robot.nextShot + " autoType " + Robot.autoType);
        System.out.println("calculated autoAngleDegrees " + Robot.wrist.absoluteToDegrees(Robot.autoWristAngleAbs));
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double minRPM;

        commandCounter++;

        if (!isFiring) {
            // In autonomous, allow robot to grab note within a specified time period, if not successful abort
            if (Robot.runningAutonomousMode && !Robot.diagnostics.getRobotHasNote()) {
                if (commandCounter > 0.2 * 50) {
                    fireCounter = 300;
                }
                return;
            }
            // mini indexer if note is in the bot but not in shooter
            if (Robot.sensors.collector() ||
                (Robot.sensors.indexer() && !Robot.sensors.shooter())) {
                Robot.indexer.setIndexer(Constants.Indexer.INDEXER_ONE_COLLECT_PERC, Constants.Indexer.INDEXER_TWO_COLLECT_PERC);
                indexingRequired = true;
                return;
            } else {
                if (indexingRequired) {
                    Robot.indexer.stopIndexerHard();
                    indexingRequired = false;
                    return;
                }
            }
        }

        // If robot has a note, set the target position to move the wrist
        // Do not move the wrist if the shooter has started to fire
        if (!isFiring && Robot.sensors.shooter()) {
            if (Robot.wrist.isAutoAngleEnabled()) {
                Robot.wrist.setWristAuto(); // will continue to adjust wrist if robot is moving
            } else {
                Robot.wrist.setTargetPosition(Robot.nextShot.getWristAngle());
            }
        }

        minRPM = Robot.wrist.isAutoAngleEnabled() ? Robot.autoShooterMinRPM : Robot.nextShot.getMinimumRPM();
        System.out.println(minRPM + " " + Robot.shooter.getShooterRPM() + " " + isFiring +" " + Robot.wrist.isWristWithinTolerance() + " " + Robot.sensors.shooter());

        if ( ((Robot.shooter.getShooterRPM() >= minRPM) &&
                    Robot.wrist.isWristWithinTolerance() &&
                    Robot.sensors.shooter()) ||  // RPM and wrist within tolerance and has note in shooter
                commandCounter >= 1.0*50     ||  // force shot after designated time
                isFiring) {                      // once the robot begins to fire, continue to fire and don't change angle
            Robot.indexer.setIndexer(0, Constants.Indexer.INDEXER_TWO_SHOOT_PERC);
            isFiring = true;
            Robot.killAutoAlign = true;
            fireCounter++;
//            System.out.println("Firing  " + minRPM + " " +
//                                            Robot.shooter.getShooterRPM() + " " +
//                                            Robot.wrist.absoluteToDegrees(Robot.autoWristAngleAbs) + " " +
//                                            Robot.wrist.isWristWithinTolerance() + " " +
//                                            Robot.sensors.shooter());
//            System.out.println("fired");
        } else {
            // not ready to fire, continue to rev the flywheel
//            System.out.println("Revving " + minRPM + " " +
//                                            Robot.shooter.getShooterRPM() + " " +
//                                            Robot.wrist.absoluteToDegrees(Robot.autoWristAngleAbs) + " " +
//                                            Robot.wrist.isWristWithinTolerance() + " " +
//                                            Robot.sensors.shooter());
            Robot.shooter.configMotorControllerAndRev();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return fireCounter > (0.25*50); // need to run the indexer for 0.25 seconds to push note through
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Total shoot counter: " + (commandCounter) + " fire only counter: " + (fireCounter) );
        Robot.indexer.stopIndexerCoast();
        Robot.shooter.setVoltagePercent(0);
        if (Robot.indexer.getDefaultCommand() == null) {
            Robot.indexer.setDefaultCommand(new IndexerDefault());
        }
        Robot.wrist.setWristCollectPosition();
    }
}
