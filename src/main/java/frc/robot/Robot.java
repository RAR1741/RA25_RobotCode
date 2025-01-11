// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.constants.RobotConstants;
import frc.robot.controls.controllers.DriverController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private final RobotConstants m_constants;
  private final ArrayList<Subsystem> m_subsystems;

  private final SwerveDrive m_swerve;
  private final RAROdometry m_odometry;
  private final DriverController m_driverController;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xRateLimiter;
  private final SlewRateLimiter m_yRateLimiter;
  private final SlewRateLimiter m_rotRateLimiter;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_subsystems = new ArrayList<>();
    m_constants = RobotConstants.getInstance();
    m_swerve = SwerveDrive.getInstance();
    m_odometry = RAROdometry.getInstance();

    m_driverController = new DriverController(0);
    m_xRateLimiter = new SlewRateLimiter(3);
    m_yRateLimiter = new SlewRateLimiter(3);
    m_rotRateLimiter = new SlewRateLimiter(3);
    
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
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis());
    double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis());
    double rot = m_rotRateLimiter.calculate(m_driverController.getTurnAxis());
    
    // slowScaler should scale between k_slowScaler and 1
    double slowScaler = RobotConstants.robotConfig.SwerveDrive.k_slowScaler + ((1 - m_driverController.getSlowScaler()) * (1 - RobotConstants.robotConfig.SwerveDrive.k_slowScaler));
    // boostScaler should scale between 1 and k_boostScaler
    double boostScaler = 1 + (m_driverController.getBoostScaler() * (RobotConstants.robotConfig.SwerveDrive.k_boostScaler - 1));

    xSpeed *= slowScaler * boostScaler;
    ySpeed *= slowScaler * boostScaler;
    rot *= slowScaler * boostScaler;

    m_swerve.drive(xSpeed,ySpeed,rot, false);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
