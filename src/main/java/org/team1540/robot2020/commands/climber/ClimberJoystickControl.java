package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.SlewRisingRateLimiter;

public class ClimberJoystickControl extends CommandBase {

    private final SlewRisingRateLimiter slewRisingRateLimiter = new SlewRisingRateLimiter(1); // TODO tune rate limiter
    private Climber climber;
    private ChickenXboxController.Axis axis;


    public ClimberJoystickControl(Climber climber, ChickenXboxController.Axis axis) {
        this.climber = climber;
        this.axis = axis;
    }

    @Override
    public void initialize() {
        slewRisingRateLimiter.reset(0);
    }

    @Override
    public void execute() {
        climber.setPercent(slewRisingRateLimiter.calculate(axis.withDeadzone(0.12).value()));
    }
}
