package org.team1540.robot2020.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakePercent extends CommandBase {
    private Intake intake;
    private double targetPercent;

    public IntakePercent(Intake intake, double targetPercent) {
        this.intake = intake;
        this.targetPercent = targetPercent;
        addRequirements(intake);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setPercent(targetPercent);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPercent(0);
    }
}
