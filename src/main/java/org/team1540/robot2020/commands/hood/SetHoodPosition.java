package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Hood;

public class SetHoodPosition extends CommandBase {
    private final double position;
    private Hood hood;

    public SetHoodPosition(Hood hood, double position) {
        this.hood = hood;
        this.position = position;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setPosition(this.position);
    }
}
