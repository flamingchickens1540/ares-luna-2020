/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team1540.robot2020;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.apache.log4j.Logger;

public class Robot extends TimedRobot {
  private static final Logger logger = Logger.getLogger(Robot.class);

  private RobotContainer container;

  @Override
  public void robotInit() {
    logger.info("Initializing FRC Team 1540 Ares/Luna Robot Code...");
    var start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    container = new RobotContainer();

    logger.info("Setting up command logging hooks...");
    CommandScheduler.getInstance().onCommandInitialize(command -> {
      logger.debug("Starting command: " + command.getName() + " " + command.getRequirements());
    });
    CommandScheduler.getInstance().onCommandFinish(command -> {
      logger.debug("Command finished: " + command.getName());
    });
    CommandScheduler.getInstance().onCommandInterrupt(command -> {
      logger.debug("Command interrupted: " + command.getName());
    });

    var end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
    logger.info("Robot ready. (" + (end - start) + "ms)");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }
}
