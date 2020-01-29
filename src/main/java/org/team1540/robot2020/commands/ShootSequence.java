package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.commands.indexer.MoveBallsToTop;
import org.team1540.robot2020.commands.indexer.MoveBallsUpOne;
import org.team1540.robot2020.commands.indexer.MoveIndexerDownUntilNotTripped;
import org.team1540.robot2020.commands.indexer.MoveIndexerUpUntilTripped;
import org.team1540.robot2020.commands.intake.IntakeIn;
import org.team1540.robot2020.commands.intake.IntakeOut;
import org.team1540.robot2020.commands.shooter.SpinUpShooter;
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
        // TODO: why not make this class a command group?
        new ParallelCommandGroup(
                new SpinUpShooter(shooter, 2000),
                new MoveBallsToTop(indexer)
        ).asProxy().schedule();
    }

    @Override
    public void execute() {
        // TODO: should be a command group, you can't have commands run sequentially by doing this

        if (indexer.isFull()) {
            new IntakeOut(intake).schedule();
        } else {
            new IntakeIn(intake).schedule();
        }
        new WaitForShooterUpToSpeed(shooter).asProxy().schedule();
        new MoveBallsUpOne(indexer, 1.5).asProxy().schedule();
        new WaitCommand(0.1).asProxy().schedule();
        indexer.ballRemoved();
        if (indexer.getShooterStaged()) {
            // TODO: These could be combined into one regular command, since they both line up the balls in preparation for shooting
            new MoveIndexerDownUntilNotTripped(indexer).asProxy().schedule();
        } else {
            new MoveIndexerUpUntilTripped(indexer).asProxy().schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: Would be better to have a stopshooter method in the shooter class
        shooter.stop();
    }
}
