package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.commands.indexer.MoveBallsToTop;
import org.team1540.robot2020.commands.indexer.MoveBallsUp;
import org.team1540.robot2020.commands.indexer.MoveIndexerDownUntilNotTripped;
import org.team1540.robot2020.commands.indexer.MoveIndexerUpUntilTripped;
import org.team1540.robot2020.commands.intake.IntakeIn;
import org.team1540.robot2020.commands.intake.IntakeOut;
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
        new ParallelCommandGroup(
                new SpinUpShooter(shooter, 2000),
                new MoveBallsToTop(indexer)
        ).asProxy().schedule();
    }

    @Override
    public void execute() {
        if (indexer.isFull()) {
            new IntakeOut(intake).schedule();
        } else {
            new IntakeIn(intake).schedule();
        }
        new WaitForShooterUpToSpeed(shooter).asProxy().schedule();
        new MoveBallsUp(indexer, 1.5).asProxy().schedule();
        new WaitCommand(0.1).asProxy().schedule();
        indexer.ballRemoved();
        if (indexer.getShooterStaged()) {
            new MoveIndexerDownUntilNotTripped(indexer).asProxy().schedule();
        } else {
            new MoveIndexerUpUntilTripped(indexer).asProxy().schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        new StopShooter(shooter).schedule();
    }
}
