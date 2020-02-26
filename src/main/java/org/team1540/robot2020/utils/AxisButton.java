package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A button based on a joystick axis. This can be used to emulate a button using a trigger or
 * joystick.
 */
public class AxisButton extends Button {

    private ChickenXboxController.Axis axis;
    private double threshold;

    /**
     * Constructs an {@code AxisButton}.
     *
     * @param axis The axis
     * @param threshold The threshold for the button to be triggered
     */
    public AxisButton(ChickenXboxController.Axis axis, double threshold) {
        this.axis = axis;
        this.threshold = threshold;
    }

    @Override
    public boolean get() {
        return (Math.abs(axis.value()) >= Math.abs(threshold)
                && Math.signum(axis.value()) == Math.signum(threshold));
    }
}
