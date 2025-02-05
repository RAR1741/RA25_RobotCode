// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.Task;
import frc.robot.constants.RobotConstants;
import frc.robot.controls.controllers.DriverController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private final ArrayList<Subsystem> m_subsystems;

  private final SwerveDrive m_swerve;
  private final RAROdometry m_odometry;
  private final DriverController m_driverController;

  private final AutoRunner m_autoRunner;
  private final AutoChooser m_autoChooser;
  private Task m_currentTask;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    RobotConstants.getInstance();
    m_subsystems = new ArrayList<>();
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();
    m_autoRunner = AutoRunner.getInstance();
    m_autoChooser = AutoChooser.getInstance();

    m_driverController = new DriverController(0, false, false, 0.5);
    m_subsystems.add(m_swerve);
    m_subsystems.add(m_odometry);
  }

  @Override
  public void robotInit() {
    new RobotTelemetry();

    // Initialize on-board logging
    DataLogManager.start();
    RobotTelemetry.print("Logging Initialized. Fard.");
  }

  @Override
  public void robotPeriodic() {
    m_subsystems.forEach(subsystem -> subsystem.periodic());
    m_subsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    m_subsystems.forEach(subsystem -> subsystem.writeToLog());
  }

  @Override
  public void autonomousInit() {
    m_currentTask = m_autoRunner.getNextTask();

    // Start the first task
    if (m_currentTask != null) {
      m_currentTask.prepare();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // If there is a current task, run it
    if (m_currentTask != null) {
      // Run the current task
      m_currentTask.update();
      m_currentTask.updateSim();

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) {
          m_currentTask.prepare();
        }
      }
    }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = m_driverController.getForwardAxis() * RobotConstants.robotConfig.SwerveDrive.k_maxSpeed;
    double ySpeed = m_driverController.getStrafeAxis() * RobotConstants.robotConfig.SwerveDrive.k_maxSpeed;
    double rot = m_driverController.getTurnAxis() * RobotConstants.robotConfig.SwerveDrive.k_maxAngularSpeed;

    // slowScaler should scale between k_slowScaler and 1
    double slowScaler = RobotConstants.robotConfig.SwerveDrive.k_slowScaler
        + ((1 - m_driverController.getSlowScaler()) * (1 - RobotConstants.robotConfig.SwerveDrive.k_slowScaler));
    // boostScaler should scale between 1 and k_boostScaler
    double boostScaler = 1
        + (m_driverController.getBoostScaler() * (RobotConstants.robotConfig.SwerveDrive.k_boostScaler - 1));

    xSpeed *= slowScaler * boostScaler;
    ySpeed *= slowScaler * boostScaler;
    rot *= slowScaler * boostScaler;

    m_swerve.drive(xSpeed, ySpeed, rot, true);
    // m_swerve.drive(1, 0, 0, false);
  }

  @Override
  public void disabledInit() {
    if (DriverStation.getMatchType() == MatchType.None) {
      m_autoRunner.initialize();
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
