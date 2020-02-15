package org.team1540.robot2020.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeRun extends CommandBase {
    private Intake intake;
    private double targetRPM;

    public IntakeRun(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        SmartDashboard.putNumber("intake/RPMTarget", 5000);
    }

    @Override
    public void initialize() {
        targetRPM = SmartDashboard.getNumber("intake/RPMTarget", 0);
    }

    @Override
    public void execute() {
        intake.setVelocity(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPercent(0);
    }
}
