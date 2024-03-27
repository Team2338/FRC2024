package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorDefault extends Command {
    double counter;
    boolean limelightDetectedOrExtension;

    public CollectorDefault() {
        super();
        addRequirements(Robot.collector);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        limelightDetectedOrExtension = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        boolean collect = false;
        boolean reverse = false;

        // for when the note goes undetected under the frame, provide detection extension for a time period
//        if (false) { //Robot.limelightCollector.hasTarget()) {
//            limelightDetectedOrExtension = true;
//            counter = 0;
//        } else {
//            if (limelightDetectedOrExtension) {
//                counter++;
//                if (counter > 0.5 * 50) {
//                    limelightDetectedOrExtension = false;
//                }
//            }
//        }

        //-------- Possible States --------
        // 1. Only Collector Sensor - Run Collector and Stage 1 Indexer
        // 2. Collector Sensor and Stage 1 Sensor - Run Collector, Stage 1-2 Indexer
        // 3. Stage 1 Sensor - Run Stage 1-2 Indexer
        // 4. Stage 1 Sensor and Shooter Sensor - Nothing
        // 5. Shooter Sensor - Nothing
        // 6. Collector Sensor and Shooter Sensor - Reverse Collector and Stage 1 Indexer (?)

//        if (limelightDetectedOrExtension) {
//            collect = true;
//            reverse = false;
//        }

        // 1
        if (Robot.sensorMonitor.getCollectorSensorState() && !Robot.sensorMonitor.getIndexerSensorState() && !Robot.sensorMonitor.getShooterSensorState()) {
            collect = true;
            reverse = false;
        }

        // 2
        if (Robot.sensorMonitor.getCollectorSensorState() && Robot.sensorMonitor.getIndexerSensorState() && !Robot.sensorMonitor.getShooterSensorState()) {
            collect = true;
            reverse = false;
        }

        // 3-5
        if (!Robot.sensorMonitor.getCollectorSensorState()) {
            collect = false;
            reverse = false;
        }

        // 6
        if (Robot.sensorMonitor.getCollectorSensorState() && !Robot.sensorMonitor.getIndexerSensorState() && Robot.sensorMonitor.getShooterSensorState()) {
            collect = true;
            reverse = true;
        }

        //Always run in autonomous mode, unless we have a note
        if (Robot.runningAutonomousMode) {
            collect = !Robot.sensorMonitor.getIndexerSensorState() && !Robot.sensorMonitor.getShooterSensorState();
            reverse = false;
        }

        if (collect && reverse) {
            Robot.collector.reverse();
        } else if (collect) {
            Robot.collector.collect();
        } else {
            Robot.collector.eject();
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
