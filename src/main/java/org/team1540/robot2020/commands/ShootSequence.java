package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.commands.shooter.SpinUpShooter;
import org.team1540.robot2020.commands.shooter.StopShooter;
import org.team1540.robot2020.commands.shooter.WaitForShooterUpToSpeed;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.subsystems.Intake;
import org.team1540.robot2020.subsystems.Shooter;

public class ShootSequence extends CommandBase {
    Intake intake;
    Indexer indexer;
    Shooter shooter;

    public ShootSequence(Intake intake, Indexer indexer, Shooter shooter) {
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(intake, indexer, shooter);
    }

    @Override
    public void initialize() {
        new SpinUpShooter(shooter).asProxy().schedule();
    }

    @Override
    public void execute() {
        new WaitForShooterUpToSpeed(shooter).asProxy().schedule();

    }

    @Override
    public void end(boolean interrupted) {
        new StopShooter(shooter).schedule();
    }
}
