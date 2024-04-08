package team.gif.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class LEDSubsystemDefault extends Command {

    private int flashCounter;

    public LEDSubsystemDefault() {
        super();
        addRequirements(Robot.ledSubsystem);
    }

    // Called when the command is initially scheduled.
    public void initialize() {
        flashCounter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    public void execute() {
        double flashLength = 0.25; // in seconds, used as a timer for flashing when we are collecting

        //
        // set the note colors
        //

        // Solid Blue: Collecting
        // Flashing Blue: Indexing
        // Solid Green: Ready to shoot

        // if the robot is attempting to collect and does not have a game piece, flash the LEDs
        if (Robot.collector.getPreSensorState() || Robot.sensors.collector() || (Robot.sensors.indexer() && !Robot.sensors.shooter())) {
            flashCounter++;
            if ( flashCounter < flashLength/2*50) {
                Robot.ledSubsystem.setNoteCollecting(); // first half of flash length
            } else {
                Robot.ledSubsystem.setNoteCollectingOff(); // second half of flash length
            }
            if (flashCounter >= flashLength*50){ // need to reset the flash counter at the end of the length
                flashCounter = 0;
            }
        } else if (Robot.collector.getCollectingState() && (!Robot.sensors.collector() || !Robot.collector.getPreSensorState())) {
            Robot.ledSubsystem.setNoteCollecting();
        } else {
            if (Robot.sensors.shooter()) {
                Robot.ledSubsystem.setNoteCollected();
            } else {
                Robot.ledSubsystem.setNoteEmpty();
            }
            flashCounter = 0;
        }

        //
        // Set the Stage colors
        //

        if( !Robot.ledSubsystem.getAutoAlignFlash()) {
            if (Robot.diagnostics.getSafeToDriveUnderStage()) {
                Robot.ledSubsystem.setStageSafe();
            } else {
                Robot.ledSubsystem.setStageAvoid();
            }
        }

        Robot.ledSubsystem.setColors();
    }

    // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    public void end(boolean interrupted) {}
}
