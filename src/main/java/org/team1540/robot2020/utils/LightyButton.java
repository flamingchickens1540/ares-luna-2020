/*
 * LightyButton.java
 *
 * The most important class on the robot.
 *
 * @author Nate Sales
 */


package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LightyButton {
    private DigitalOutput led;
    private DigitalInput contact;

    public LightyButton(int ledChannel, int contactChannel) {
        led = new DigitalOutput(ledChannel);
        contact = new DigitalInput(contactChannel);
        led.set(false); // Disable the LED
    }

    public boolean isPressed() {
        return contact.get();
    }

    public void on() {
        led.set(false);
    }

    public void off() {
        led.set(true);
    }
}
