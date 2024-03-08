package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class LEDSubsystem extends SubsystemBase {
    private static AddressableLED led;
    private static AddressableLEDBuffer ledBuffer;

    private static boolean autoAlignFlash;

    private final int [] Note = {0,0,0}; // array are colors
    private final int [] Stage = {0,0,0};

    public LEDSubsystem() {
        super();
        autoAlignFlash = false;

        //initializes by inputting PWM port number from Roborio
        led = new AddressableLED(RobotMap.LED_PWM_PORT);

        //initialize by inputting length of LED (# of LED)
        ledBuffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS_TOTAL);
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
    }

    public void setNoteCollecting() {
        Note[0] = 0;
        Note[1] = 255;
        Note[2] = 120;
    }

    public void setNoteCollectingOff() {
        Note[0] = 0;
        Note[1] = 0;
        Note[2] = 0;
    }

    public void setNoteCollected() {
        Note[0] = 0;
        Note[1] = 255;
        Note[2] = 0;
    }

    public void setNoteEmpty() {
        Note[0] = 0;
        Note[1] = 0;
        Note[2] = 255;
    }
    /**
     * Assigns the color of the LEDs to green to indicate clear to go under stage
     */
    public void setStageSafe() {
        Stage[0] = 0;
        Stage[1] = 255;
        Stage[2] = 0;
    }

    /**
     * Assigns the color of the LEDs to green to indicate NOT clear to go under stage
     */
    public void setStageAvoid() {
        Stage[0] = 255;
        Stage[1] = 0;
        Stage[2] = 0;
    }

    /**
     * Turns the LEDs off
     */
    public void setLEDOff() {
        Stage[0] = 0;
        Stage[1] = 0;
        Stage[2] = 0;
    }

    /**
     * Sets the colors of the LEDs
     */
    public void setColors() {
        for (int i = 0; i < RobotMap.NOTE_LEDS.length; i++) {
            ledBuffer.setRGB(RobotMap.NOTE_LEDS[i],Note[0],Note[1], Note[2]);
            led.setData(ledBuffer);
        }
        for (int j = 0; j < RobotMap.STAGE_LEDS.length; j++) {
            ledBuffer.setRGB(RobotMap.STAGE_LEDS[j], Stage[0], Stage[1], Stage[2]);
            led.setData(ledBuffer);
        }
    }

    public boolean getAutoAlignFlash() {
        return autoAlignFlash;
    }

    public void setAutoAlignFlash(boolean flashing) {
        autoAlignFlash = flashing;
    }

}