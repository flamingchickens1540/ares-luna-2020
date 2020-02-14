package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerMoveToPosition;
import org.team1540.robot2020.commands.indexer.IndexerStageBallsForShooting;
import org.team1540.robot2020.commands.intake.Intake;

public class ShootSequence extends SequentialCommandGroup {
    private Shooter shooter;

    public ShootSequence(Intake intake, Indexer indexer, Shooter shooter) {
        this.shooter = shooter;
        addRequirements(indexer);
        addCommands(
                parallel(
                        // TODO this parallel command group should also move the shooter hood
                        new ShooterSpinUp(shooter),
                        new IndexerStageBallsForShooting(indexer)
                ),
                // TODO the flywheel needs to continue spinning while moving the balls up one and continue for a little while afterwards, we don't want to trigger the motor safety cutoff
                new IndexerMoveToPosition(indexer, () -> indexer.getPositionMeters() + 0.18, 0.5, 0.001),
                new WaitCommand(0.1)
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disableMotors();
    }
}
