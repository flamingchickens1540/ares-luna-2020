package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.LightyButton;

public class LightyButtonTest extends CommandBase {
    LightyButton lightyButton;
    Timer timer;

    @Override
    public void initialize() {
        lightyButton = new LightyButton(3, 4);
        timer = new Timer();
        lightyButton.on();
    }

    @Override
    public void execute() { // TODO: This is very bad. Just a test.
        if (lightyButton.isPressed()) {
            lightyButton.on();
        } else {
            lightyButton.off();
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
