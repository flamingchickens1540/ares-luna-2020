package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.util.ControlUtils;

import java.util.HashSet;
import java.util.Set;
import org.team1540.rooster.util.MiniPID;
import org.team1540.rooster.util.TrigUtils;
import org.team1540.rooster.wrappers.Limelight;

public class TankDrive implements Command {
    private DriveTrain driveTrain;
    private ChickenXboxController driver;
    private final Limelight limelight;
    private MiniPID pointController = new MiniPID(0, 0, 0);

    public TankDrive(DriveTrain driveTrain, ChickenXboxController driver, Limelight limelight) {
        this.driveTrain = driveTrain;
        this.driver = driver;
        this.limelight = limelight;
        SmartDashboard.putNumber("Shooter/P", 5);
        SmartDashboard.putNumber("Shooter/I", 1);
        SmartDashboard.putNumber("Shooter/D", 24);
        SmartDashboard.putNumber("Shooter/IMax", 0.1);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("Shooter/P", 0);
        double i = SmartDashboard.getNumber("Shooter/I", 0);
        double d = SmartDashboard.getNumber("Shooter/D", 0);
        double IMax = SmartDashboard.getNumber("Shooter/IMax", 0);
        pointController.setPID(p, i, d);
        pointController.setOutputLimits(0.3);
        pointController.setMaxIOutput(IMax);
    }

    @Override
    public void execute() {
        double leftMotors = ControlUtils.deadzone(driver.getRectifiedX(Hand.kLeft), 0.1);
        double rightMotors = ControlUtils.deadzone(driver.getRectifiedX(Hand.kRight), 0.1);
        if (limelight.isTargetFound()) {
            double limelightValue = NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(0) / 100;
            double pidOutput = pointController.getOutput(limelightValue);
            leftMotors -= pidOutput;
            rightMotors += pidOutput;
        }

        driveTrain.setThrottle(
            leftMotors,
            rightMotors
        );
    }

    @Override
    public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> requirements = new HashSet<>();
        requirements.add(driveTrain);
        return requirements;
    }
}
