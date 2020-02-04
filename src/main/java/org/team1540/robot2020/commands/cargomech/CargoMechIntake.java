package org.team1540.robot2020.commands.cargomech;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.team1540.robot2020.subsystems.CargoMech;
import org.team1540.rooster.util.ChickenXboxController;

public class CargoMechIntake extends CommandBase {
    private CargoMech cargoMech;
    private JoystickButton stopButton;

    public CargoMechIntake(CargoMech cargoMech, JoystickButton stopButton) {
        this.cargoMech = cargoMech;
        this.stopButton = stopButton;
    }

    @Override
    public void initialize() {
        cargoMech.setRollerSpeed(1);
    }

    @Override
    public boolean isFinished() {
        return stopButton.get();
    }

    @Override
    public void end(boolean interrupted) {
        cargoMech.setRollerSpeed(0);
        if (!interrupted) {
            this.schedule();
        }
    }
}
