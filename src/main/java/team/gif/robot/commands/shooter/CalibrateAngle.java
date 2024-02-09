package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CalibrateAngle extends Command {
    enum test {
        LOWER_LIMIT, // finds the hard limit
        CLOCKWISE_ROTATION,
        SET_ZERO,
        TEST_MIN, // this is testing the software limit, not the hard limit
        TEST_90,
        PAUSE,
        TEST_MIN_FINAL,
        FINISHED
    }

    double  zeroOffset; // this is the value we eventually save to the config
    test    testingStage;
    double  prevPos;
    int     stallCount;
    int     moveCount;
    float   pauseCounter;

    String  nl = System.lineSeparator();

    public CalibrateAngle() {
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        testingStage = test.LOWER_LIMIT;
        prevPos = -2;     // start at a number away from 0 ... 1
        zeroOffset = -1;  // start at a number away from 0 ... 1
        moveCount = 0;
        stallCount = 0;
        pauseCounter = 0;

        // reset the zero offset to hard absolute
        Robot.shooter.setZeroOffset(0);

        printStatus("********************************");
        printStatus("Calibration Started");
        printStatus("Moving to lower hard limit ...");
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double pos = Robot.shooter.getPosition();

        double speedUpClock = 0.1;
        double speedDownCounter = 0.015;

        //
        // Stage: CLOCKWISE_ROTATION
        // Test to make sure the motor can turn the shooter clockwise
        // and it is not stuck/stalled. There should be plenty of room
        // for the shooter to move
        //
        if (testingStage == test.CLOCKWISE_ROTATION) {
            Robot.shooter.moveAnglePercentPower(speedUpClock);
            if (Math.abs(prevPos - pos) < (1.0 * 1 / 4096)) { // Allows for range of 3 ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
                moveCount++;
            }

            if (stallCount == 4) {
                // The shooter should be able to move, so we
                // should never enter this loop. Indicates the
                // shooter is stuck.
                Robot.shooter.moveAnglePercentPower(0);

                System.out.println(" **************************************" + nl +
                                   " **   Error: Unable to calibrate!!   **" + nl +
                                   " ** The shooter appears to be stuck  **" + nl +
                                   " **     during clockwise test        **" + nl +
                                   " **                                  **" + nl +
                                   " **      Calibration aborted         **" + nl +
                                   " **************************************");
                testingStage = test.FINISHED;
            } else {
                moveCount++;

                if (moveCount >= 1.0 * 50) { // rotate for XX seconds
                    // The shooter has been able to move continuously
                    // Move to next step
                    Robot.shooter.moveAnglePercentPower(0);
                    printStatus("Clockwise Test passed");
                    testingStage = test.SET_ZERO;
                    stallCount = 0;
                }
            }
        }

        //
        // Stage: LOWER_LIMIT
        // Slowly move to the lower hard limit until the
        // motor stalls
        //
        if (testingStage == test.LOWER_LIMIT) {
            Robot.shooter.moveAnglePercentPower(-speedDownCounter); // turn counterclockwise very slowly

            // if collector hasn't moved, increase stall count eventually indicating we are at hard limit
            if (Math.abs(prevPos - pos) < (2.0 * 1 / 4096)) { // 3 = min ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
            }
            if (stallCount == 4) {
                Robot.shooter.moveAnglePercentPower(0); // stop the motor
                zeroOffset = -1 * (pos - 0.1);     // calculate zero offset
                printStatus("Found lower hard limit");
                printStatus("Testing ability to rotate clockwise ...");
                testingStage = test.CLOCKWISE_ROTATION;
                stallCount = 0;
            }
        }

        //
        // Stage: SET_ZERO
        // Set the zero offset
        //
        if (testingStage == test.SET_ZERO) {
            Robot.shooter.setZeroOffset(zeroOffset); // save the zero offset found in LOWER_LIMIT test
            printStatus("Zero offset set at: " + zeroOffset);
            printStatus("Testing new offset (going to soft limit) ...");
            testingStage = test.TEST_MIN;
            stallCount = 0;
        }

        //
        // Stage: TEST_MIN_FINAL
        // Second test moving to min (not hard min). It should not stall.
        //
        if (testingStage == test.TEST_MIN_FINAL) {
            Robot.shooter.moveAnglePercentPower(-speedDownCounter);

            if (pos <= Constants.Shooter.MIN_LIMIT_ABSOLUTE) {
                Robot.shooter.moveAnglePercentPower(0);
                System.out.println(" *****************************************************" + nl +
                                   " **   Calibration COMPLETE! "                     + nl +
                                   " **"                                              + nl +
                                   " **   Shooter should be at min shooting position" + nl +
                                   " **   New Zero Offset value is " + zeroOffset     + nl +
                                   " **      Update Constants.java file"              + nl +
                                   " **      Calibration COMPLETE! "                  + nl +
                                   " ****************************************************");

                testingStage = test.FINISHED;
            }

            // check for stalling motor
            if (Math.abs(pos - prevPos) < (2.0 * 1 / 4096)) { // Allows for range of 3 ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
            }

            // The shooter should be able to move to the min position, so we
            // should never enter this loop. Indicates the shooter is stuck
            // or calibration failed.
            if (stallCount == 4) {
                Robot.shooter.moveAnglePercentPower(0);
                System.out.println(" ****************************************" + nl +
                                   " **    Error: Unable to calibrate!!    **" + nl +
                                   " **    Shooter stalled during final    **" + nl +
                                   " **        test to arrive at the       **" + nl +
                                   " **            lower limit             **" + nl +
                                   " **                                    **" + nl +
                                   " **        Calibration aborted         **" + nl +
                                   " ****************************************");

                testingStage = test.FINISHED;
            }
        }
        //
        // Stage: PAUSE
        // Pausing test to allow user to verify shooter is at 90 degrees
        //
        if (testingStage == test.PAUSE) {
            Robot.shooter.moveAnglePercentPower(0.01); // apply small power to overcome gravity
            pauseCounter++;

            if (pauseCounter >= 3 * 50){ // 5 seconds x 50 20ms intervals
                printStatus("Pause complete");
                printStatus("Testing new offset (going to soft limit) ...");
                testingStage = test.TEST_MIN_FINAL;
            }
        }

        //
        // Stage: TEST_90
        // Test moving shooter to 90 degree angle.
        // Zero offset has already been calculated and set
        // It should not stall.
        //
        if (testingStage == test.TEST_90) {
            Robot.shooter.moveAnglePercentPower(speedUpClock);

            // check for stalling motor
            if (Math.abs(pos - prevPos) < (1.0 * 1 / 4096)) { // Allows for range of 3 ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
            }

            if (pos >= Robot.shooter.degreesToAbsolute(90)) {
                Robot.shooter.moveAnglePercentPower(0);
                printStatus("90 degree test complete");
                printStatus("");
                printStatus("Shooter should be pointing to 90 degrees");
                printStatus("Waiting 3 seconds to verify visually ... ");
                testingStage = test.PAUSE;
            }

            // The shooter should be able to move to the 90 degree position, so we
            // should never enter this loop. Indicates the shooter is stuck
            // or calibration failed.
            if (stallCount == 2) {
                Robot.shooter.moveAnglePercentPower(0);
                System.out.println(" ***************************************" + nl +
                                   " **    Error: Unable to calibrate!!   **" + nl +
                                   " **    Shooter stalled during test    **" + nl +
                                   " **      to arrive at 90 degrees      **" + nl +
                                   " **                                   **" + nl +
                                   " **        Calibration aborted        **" + nl +
                                   " ***************************************");
                testingStage = test.FINISHED;
            }
        }

        //
        // Stage: TEST_MIN
        // Test moving to min (not hard min). It should not stall.
        // Prior to this stage, new zero was set correctly
        //
        if (testingStage == test.TEST_MIN) {
            Robot.shooter.moveAnglePercentPower(-speedDownCounter);

            // check for stalling motor
            if (Math.abs(pos - prevPos) < (2.0 * 1.0 / 4096)) { // Allows for range of 3 ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
            }
            if (pos <= Constants.Shooter.MIN_LIMIT_ABSOLUTE) {
                Robot.shooter.moveAnglePercentPower(0);
                printStatus("Min limit test complete");
                printStatus("Moving to 90 degrees ...");
                testingStage = test.TEST_90;
            }

            // The shooter should be able to move to the min position, so we
            // should never enter this loop. Indicates the shooter is stuck
            // or calibration failed.
            if (stallCount == 4) {
                Robot.shooter.moveAnglePercentPower(0);
                System.out.println(" ****************************************" + nl +
                                   " **    Error: Unable to calibrate!!    **" + nl +
                                   " **    Shooter stalled during first    **" + nl +
                                   " **    test to arrive at the target    **" + nl +
                                   " **            min position.           **" + nl +
                                   " **                                    **" + nl +
                                   " **        Calibration aborted         **" + nl +
                                   " ****************************************");
                testingStage = test.FINISHED;
            }
        }
        prevPos = pos;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return (testingStage == test.FINISHED);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.moveAnglePercentPower(0);
    }

    void printStatus(String status) {
        System.out.println(" ** " + status );
    }
}