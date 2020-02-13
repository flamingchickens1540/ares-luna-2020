package org.team1540.robot2020.commands.cargomech;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.team1540.robot2020.subsystems.CargoMech;
import org.team1540.rooster.util.ChickenXboxController;

public class CargoMechIntake extends CommandBase {
    private CargoMech cargoMech;
    private Timer timer = new Timer();

    public CargoMechIntake(CargoMech cargoMech) {
        this.cargoMech = cargoMech;

    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        cargoMech.setRollerSpeed(1);
    }

    @Override
    public boolean isFinished() {
        return timer.hasPeriodPassed(2.5);
    }

    @Override
    public void end(boolean interrupted) {
        cargoMech.setRollerSpeed(0);
    }
}
