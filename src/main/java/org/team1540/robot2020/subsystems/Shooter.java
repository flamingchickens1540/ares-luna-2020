package org.team1540.robot2020.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private PIDController controller = new PIDController(1, 0, 0);

}
