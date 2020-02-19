package org.team1540.robot2020.commands.cargomech;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.CargoMech;
import org.team1540.rooster.util.ChickenXboxController;

public class CargoMechManualControl extends CommandBase {
    private CargoMech cargoMech;
    private ChickenXboxController.Axis joystickAxis;

    public CargoMechManualControl(CargoMech cargoMech, ChickenXboxController.Axis joystickAxis) {
        this.cargoMech = cargoMech;
        this.joystickAxis = joystickAxis;
        addRequirements(cargoMech);
    }

    @Override
    public void execute() {
        cargoMech.setRollerSpeed(joystickAxis.value());
    }

    @Override
    public void end(boolean interrupted) {
        cargoMech.setRollerSpeed(0);
    }
}
