package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.team1540.rooster.wrappers.ChickenTalon;
import org.team1540.rooster.wrappers.ChickenVictor;

public class DriveTrain implements Subsystem {
    private ChickenTalon driveLeftA = new ChickenTalon(13);
    private ChickenVictor driveLeftB = new ChickenVictor(12);
    private ChickenVictor driveLeftC = new ChickenVictor(11);

    private ChickenTalon driveRightA = new ChickenTalon(1);
    private ChickenVictor driveRightB = new ChickenVictor(2);
    private ChickenVictor driveRightC = new ChickenVictor(3);

    public DriveTrain() {
        driveLeftB.follow(driveLeftA);
        driveLeftC.follow(driveLeftA);

        driveRightB.follow(driveRightA);
        driveRightC.follow(driveRightA);
    }

    public void setThrottle(double left, double right) {
        driveLeftA.set(ControlMode.PercentOutput, -left);
        driveRightA.set(ControlMode.PercentOutput, right);
    }
}
