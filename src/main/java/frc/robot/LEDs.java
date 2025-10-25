package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A class that sets the colors on the LED strip.
 */
public class LEDs extends SubsystemBase {
    /**
     * The object that communicates with the LED driver, which is a device
     * that connects the RoboRIO's PWM port to the LEDs.
     */
    private AddressableLED addressableLeds;

    /**
     * The buffer that holds the color values for each LED.
     */
    private AddressableLEDBuffer colorBuffer;

    /**
     * The frame index, which is used to time the scrolling effect.
     */
    private int frameIndex = 0;

    /**
     * The scroll rate, in LEDs per second. If this is greater than 50, it is
     * rounded down to the nearest integer multiple of 50, otherwise its rounded
     * down to the nearest 50 / n, where n is an integer.
     */
    private double scrollSpeed = 50.0;

    /**
     * The RobotContainer instance, used to access information about the state
     * of the robot to determine what colors to display on the LED strip.
     */
    private RobotContainer robot;

    /**
     * This method (which is called the constructor), is run once when the
     * robot program starts. It initializes the LED driver and the color
     * buffer, and sets the initial colors of the LEDs to black (off).
     * 
     * This runs when the program starts because one instance of this class is
     * created when the RobotContainer class is initialized, which occurs at
     * startup.
     */
    public LEDs(RobotContainer robot) {
        /// Stores the RobotContainer instance in the robot variable.
        this.robot = robot;

        // Create the AddressableLED object, specifying the PWM port that the
        // LED driver is connected to. In this case, it's connected to PWM port
        // 9 on the RoboRIO.
        addressableLeds = new AddressableLED(9);
        
        // Set the LED driver to use the GRB color order.
        addressableLeds.setColorOrder(ColorOrder.kGRB);

        // Create the color buffer, specifying the number of LEDs in the strip.
        // In this case, there are 56 LEDs in the strip.
        colorBuffer = new AddressableLEDBuffer(56);

        // Set the length of the LED strip in the AddressableLED object to
        // match the length of the color buffer.
        addressableLeds.setLength(colorBuffer.getLength());

        // Set the initial colors of all the LEDs in the buffer to black (off).
        for (int i = 0; i < colorBuffer.getLength(); i++) {
            colorBuffer.setLED(i, new Color(0, 0, 0));
        }

        // Send the initial color buffer to the LED strip.
        addressableLeds.setData(colorBuffer);

        // Start updating the LED strip.
        addressableLeds.start();
    }

    /**
     * This method runs 50 times per second, and should be used to update the
     * colors displayed on the LED strip.
     */
    @SuppressWarnings("unused")
    @Override
    public void periodic() {
        // The time since the program started, in seconds. You can use this
        // variable to make the colors change over time.
        double time = Timer.getTimestamp();

        // Whether the robot is currently in autonomous mode.
        boolean autonomous = RobotState.isAutonomous();

        // Whether the robot is holding coral in the manipulator.
        boolean hasCoral = robot.manipulator.grabber.hasCoral();

        // Whether the robot is holding algae.
        boolean hasAlgae = robot.manipulator.grabber.hasAlgae();

        // Whether the robot is currently enabled.
        boolean enabled = RobotState.isEnabled();

        ///////////////////////// WRITE YOUR CODE BELOW ////////////////////////
        
        
        
        
        
        
        
        // How red the color should be, measured from 0.0 (not red at all) to
        // 1.0 (as red as possible).
        double red = 0.95;
        
        // How green the color should be, measured from 0.0 (not green at all)
        // to 1.0 (as green as possible).
        double green = 0.87;
        
        // How blue the color should be, measured from 0.0 (not blue at all) to
        // 1.0 (as blue as possible).
        double blue = 0.55;
        
        
        
        
        
        
        
        ///////////////////////// WRITE YOUR CODE ABOVE ////////////////////////

        // Update the colors on the LED strip using the updateColors method.
        applyColor(red, green, blue);
    }

    /**
     * Updates the colors on the LED strip, adding a new color at the bottom
     * (defined by {@code red}, {@code green}, and {@code blue}) and scrolling
     * the existing colors up the strip.
     * 
     * @param red The fraction of red in the new color, from 0.0 (no red) to
     * 1.0 (full red).
     * @param green The fraction of green in the new color, from 0.0 (no green)
     * to 1.0 (full green).
     * @param blue The fraction of blue in the new color, from 0.0 (no blue) to
     * 1.0 (full blue).
     */
    private void applyColor(double red, double green, double blue) {
        // Create Color object to store the new color that uses the provided
        // red, green, and blue values.
        Color newColor = new Color(red, green, blue);

        // Calculate the number of periodic cycles per new color based on the
        // scroll speed.
        int cyclesPerLED = (int) (50.0 / scrollSpeed);

        // Calculate the number of new LEDs that will be shown each periodic
        // cycle.
        int ledsPerCycle = (int) (scrollSpeed / 50.0);

        // This if condition checks if the scroll speed is slow enough that we
        // shouldn't scroll the LED strip by one LED every periodic cycle, or
        // if its fast enough that we should scroll by more than one LED
        // every periodic cycle.
        if (cyclesPerLED > 1) {
            // This code runs if the scroll speed is slow enough that we
            // shouldn't scroll the LED strip by one LED every periodic cycle.

            // Only scroll the LED strip by one LED every cyclesPerLED periodic
            // cycles.
            if (frameIndex % cyclesPerLED == 0) {
                // Scroll the colors by one LED and add the next color.
                scrollColors(1, newColor);
            }
        } else {
            // This code runs when the scroll speed is so fast that we should
            // scroll the LED strip by more than one LED each periodic cycle.
            scrollColors(ledsPerCycle, newColor);
        }

        frameIndex++;
    }

    /**
     * Scrolls the colors in the color buffer up by {@code ledsPerCycle} LEDs,
     * adding {@code newColor} to the bottom of the buffer.
     * 
     * @param ledsPerCycle The number of LEDs to scroll the colors by.
     * @param newColor The new color to add to the bottom of the buffer.
     */
    private void scrollColors(int ledsPerCycle, Color newColor) {
        // Use a for loop to iterate through the LEDs in reverse. In the loop,
        // colors are copied from earlier positions to later positions in the
        // buffer, so we need to start at the end of the buffer and work
        // backwards to avoid setting lower colors before we copy them to higher
        // positions.
        for (int i = colorBuffer.getLength() - 1; i >= ledsPerCycle; i--) {
            // Set the LED at position i to the color of the LED that is
            // ledsPerCycle positions earlier in the buffer.
            colorBuffer.setLED(i, colorBuffer.getLED(i - ledsPerCycle));
        }

        // Set the bottom ledsPerCycle LEDs to the new color, by iterating
        // through the first ledsPerCycle positions in the buffer and setting
        // each one to the new color.
        for (int i = 0; i < ledsPerCycle; i++) {
            // Set the LED at position i to the new color.
            colorBuffer.setLED(i, newColor);
        }

        // Send the updated color buffer to the LED strip.
        addressableLeds.setData(colorBuffer);
    }
}
