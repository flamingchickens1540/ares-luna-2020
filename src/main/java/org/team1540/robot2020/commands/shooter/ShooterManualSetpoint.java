package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ShooterManualSetpoint extends CommandBase {
    private double rpm = 5000;
    private Shooter shooter;
    private ChickenXboxController.Axis joystickAxis;

    public ShooterManualSetpoint(Shooter shooter, ChickenXboxController.Axis joystickAxis) {
        this.shooter = shooter;
        this.joystickAxis = joystickAxis;
    }

    @Override
    public void execute() {
        rpm += joystickAxis.withDeadzone(0.1).value() * 5;

        SmartDashboard.putNumber("shooter/manualSetpointRPM", rpm);
        shooter.setVelocityRPM(rpm);
    }
}
