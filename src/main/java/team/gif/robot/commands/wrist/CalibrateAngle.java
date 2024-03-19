package team.gif.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CalibrateAngle extends Command {
    enum stage {
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
    stage   testingStage;
    double  prevPos;
    int     stallCount;
    int     moveCount;
    float   pauseCounter;

    private String  nl = System.lineSeparator();

    public CalibrateAngle() {
        super();
        addRequirements(Robot.wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        testingStage = stage.LOWER_LIMIT;
        prevPos = -2;     // start at a number away from 0 ... 1
        zeroOffset = -1;  // start at a number away from 0 ... 1
        moveCount = 0;
        stallCount = 0;
        pauseCounter = 0;

        // reset the zero offset to hard absolute
        Robot.wrist.setZeroOffset(0);

        printStatus("********************************");
        printStatus("Calibration Started");
        printStatus("Moving to lower hard limit ...");
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double pos = Robot.wrist.getPosition();

        double increaseAngleSpeed = Constants.Wrist.INCREASE_ANGLE_PWR_PERC_CALIBRATION;
        double decreaseAngleSpeed = Constants.Wrist.DECREASE_ANGLE_PWR_PERC_CALIBRATION;

        //
        // Stage: CLOCKWISE_ROTATION
        // Test to make sure the motor can turn the wrist clockwise
        // and it is not stuck/stalled. There should be plenty of room
        // for the wrist to move
        //
        if (testingStage == stage.CLOCKWISE_ROTATION) {
            Robot.wrist.moveWristPercentPower(increaseAngleSpeed);
            if (Math.abs(prevPos - pos) < (1.0 * 1 / 4096)) { // Allows for range of 3 ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
                moveCount++;
            }

            if (stallCount == 4) {
                // The wrist should be able to move, so we
                // should never enter this loop. Indicates the
                // wrist is stuck.
                Robot.wrist.moveWristPercentPower(0);

                System.out.println(" **************************************" + nl +
                                   " **   Error: Unable to calibrate!!   **" + nl +
                                   " ** The wrist appears to be stuck  **" + nl +
                                   " **     during clockwise test        **" + nl +
                                   " **                                  **" + nl +
                                   " **      Calibration aborted         **" + nl +
                                   " **************************************");
                testingStage = stage.FINISHED;
            } else {
                moveCount++;

                if (moveCount >= 1.0 * 50) { // rotate for XX seconds
                    // The wrist has been able to move continuously
                    // Move to next step
                    Robot.wrist.moveWristPercentPower(0);
                    printStatus("Clockwise Test passed");
                    testingStage = stage.SET_ZERO;
                    stallCount = 0;
                }
            }
        }

        //
        // Stage: LOWER_LIMIT
        // Slowly move to the lower hard limit until the
        // motor stalls
        //
        if (testingStage == stage.LOWER_LIMIT) {
            Robot.wrist.moveWristPercentPower(-decreaseAngleSpeed); // turn counterclockwise very slowly

            // if collector hasn't moved, increase stall count eventually indicating we are at hard limit
            if (Math.abs(prevPos - pos) < (2.0 * 1 / 4096)) { // 3 = min ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
            }
            if (stallCount == 4) {
                Robot.wrist.moveWristPercentPower(0); // stop the motor
                zeroOffset = -1 * (pos - 0.1);     // calculate zero offset
                printStatus("Found lower hard limit");
                printStatus("Testing ability to rotate clockwise ...");
                testingStage = stage.CLOCKWISE_ROTATION;
                stallCount = 0;
            }
        }

        //
        // Stage: TEST_MIN
        // Test moving to min (not hard min). It should not stall.
        // Prior to this stage, new zero was set correctly
        //
        if (testingStage == stage.TEST_MIN) {
            Robot.wrist.moveWristPercentPower(-decreaseAngleSpeed);

            // check for stalling motor
            if (Math.abs(pos - prevPos) < (2.0 * 1.0 / 4096)) { // Allows for range of 3 ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
            }
            if (pos <= Constants.Wrist.MIN_LIMIT_ABSOLUTE) {
                Robot.wrist.moveWristPercentPower(0);
                printStatus("Min limit test complete");
                printStatus("Moving to 90 degrees ...");
                testingStage = stage.TEST_90;
            }

            // The wrist should be able to move to the min position, so we
            // should never enter this loop. Indicates the wrist is stuck
            // or calibration failed.
            if (stallCount == 4) {
                Robot.wrist.moveWristPercentPower(0);
                System.out.println(" ****************************************" + nl +
                                   " **    Error: Unable to calibrate!!    **" + nl +
                                   " **    Wrist stalled during first      **" + nl +
                                   " **    test to arrive at the target    **" + nl +
                                   " **            min position.           **" + nl +
                                   " **                                    **" + nl +
                                   " **        Calibration aborted         **" + nl +
                                   " ****************************************");
                testingStage = stage.FINISHED;
            }
        }

        //
        // Stage: SET_ZERO
        // Set the zero offset
        //
        if (testingStage == stage.SET_ZERO) {
            Robot.wrist.setZeroOffset(zeroOffset); // save the zero offset found in LOWER_LIMIT test
            printStatus("Zero offset set at: " + zeroOffset);
            printStatus("Testing new offset (going to soft limit) ...");
            testingStage = stage.TEST_MIN;
            stallCount = 0;
        }

        //
        // Stage: TEST_MIN_FINAL
        // Second test moving to min (not hard min). It should not stall.
        //
        if (testingStage == stage.TEST_MIN_FINAL) {
            Robot.wrist.moveWristPercentPower(-decreaseAngleSpeed*1.5);

            if (pos <= Constants.Wrist.MIN_LIMIT_ABSOLUTE) {
                Robot.wrist.moveWristPercentPower(0);
                System.out.println(" *****************************************************" + nl +
                                   " **   Calibration COMPLETE! "                     + nl +
                                   " **"                                              + nl +
                                   " **   Wrist should be at min shooting position" + nl +
                                   " **   New Zero Offset value is " + zeroOffset     + nl +
                                   " **      Update Constants.java file"              + nl +
                                   " **      Calibration COMPLETE! "                  + nl +
                                   " ****************************************************");

                testingStage = stage.FINISHED;
            }

            // check for stalling motor
            if (Math.abs(pos - prevPos) < (2.0 * 1 / 4096)) { // Allows for range of 3 ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
            }

            // The wrist should be able to move to the min position, so we
            // should never enter this loop. Indicates the wrist is stuck
            // or calibration failed.
            if (stallCount == 4) {
                Robot.wrist.moveWristPercentPower(0);
                System.out.println(" ****************************************" + nl +
                                   " **    Error: Unable to calibrate!!    **" + nl +
                                   " **    Wrist stalled during final    **" + nl +
                                   " **        test to arrive at the       **" + nl +
                                   " **            lower limit             **" + nl +
                                   " **                                    **" + nl +
                                   " **        Calibration aborted         **" + nl +
                                   " ****************************************");

                testingStage = stage.FINISHED;
            }
        }
        //
        // Stage: PAUSE
        // Pausing test to allow user to verify wrist is at 90 degrees
        //
        if (testingStage == stage.PAUSE) {
            Robot.wrist.holdWrist();
            pauseCounter++;

            if (pauseCounter >= 0.5 * 50){ // 5 seconds x 50 20ms intervals
                printStatus("Pause complete");
                printStatus("Testing new offset (going to soft limit) ...");
                testingStage = stage.TEST_MIN_FINAL;
            }
        }

        //
        // Stage: TEST_90
        // Test moving wrist to 90 degree angle.
        // Zero offset has already been calculated and set
        // It should not stall.
        //
        if (testingStage == stage.TEST_90) {
            Robot.wrist.moveWristPercentPower(increaseAngleSpeed*1.5);

            // check for stalling motor
            if (Math.abs(pos - prevPos) < (1.0 * 1 / 4096)) { // Allows for range of 3 ticks to check, 4096 is number of ticks in 1 rotation
                stallCount++;
            } else {
                stallCount = 0;
            }

            if (pos >= Robot.wrist.degreesToAbsolute(90)) {
                Robot.wrist.moveWristPercentPower(0);
                printStatus("90 degree test complete");
                printStatus("");
                printStatus("Wrist should be pointing to 90 degrees");
                printStatus("Waiting 3 seconds to verify visually ... ");
                testingStage = stage.PAUSE;
            }

            // The wrist should be able to move to the 90 degree position, so we
            // should never enter this loop. Indicates the wrist is stuck
            // or calibration failed.
            if (stallCount == 2) {
                Robot.wrist.moveWristPercentPower(0);
                System.out.println(" ***************************************" + nl +
                                   " **    Error: Unable to calibrate!!   **" + nl +
                                   " **    Wrist stalled during test    **" + nl +
                                   " **      to arrive at 90 degrees      **" + nl +
                                   " **                                   **" + nl +
                                   " **        Calibration aborted        **" + nl +
                                   " ***************************************");
                testingStage = stage.FINISHED;
            }
        }

        prevPos = pos;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return (testingStage == stage.FINISHED);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.wrist.moveWristPercentPower(0);
        Robot.wrist.setTargetPosition(Robot.wrist.getPosition());
    }

    void printStatus(String status) {
        System.out.println(" ** " + status );
    }
}