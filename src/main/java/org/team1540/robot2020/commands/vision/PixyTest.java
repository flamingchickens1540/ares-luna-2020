package org.team1540.robot2020.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

import java.util.ArrayList;

public class PixyTest extends CommandBase {

    private Pixy2 pixy;

    public PixyTest(Pixy2 camera) {
        this.pixy = camera;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("pixyCam/servoX", 500);
        SmartDashboard.putNumber("pixyCam/servoY", 800);
    }

    @Override
    public void execute() {
        int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 5);
        ArrayList<Pixy2CCC.Block> blocks = pixy.getCCC().getBlocks();
        SmartDashboard.putNumber("pixyCam/numTargets", blocks.size());
        for (int i = 0; i < blocks.size(); i++) {
            SmartDashboard.putNumber("pixyCam/cccTargets/target" + i + "/x", blocks.get(i).getY());
            SmartDashboard.putNumber("pixyCam/cccTargets/target" + i + "/y", blocks.get(i).getX());
        }

        int servoX = (int) Math.round(SmartDashboard.getNumber("pixyCam/servoX", 0));
        int servoY = (int) Math.round(SmartDashboard.getNumber("pixyCam/servoY", 0));

        pixy.setServos(servoX, servoY);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
