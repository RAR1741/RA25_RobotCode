// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.ArmTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ElevatorTask;
import frc.robot.autonomous.tasks.EndEffectorTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PrintTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.constants.RobotConstants;
import frc.robot.controls.controllers.DriverController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.controls.controllers.FilteredController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.TaskScheduler;
import frc.robot.subsystems.drivetrain.RAROdometry;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.intakes.Intakes;
import frc.robot.subsystems.intakes.Intake.IntakeState;
import frc.robot.subsystems.intakes.Intakes.IntakeVariant;
import frc.robot.controls.controllers.VirtualRobotController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorState;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.PoseAligner;
import frc.robot.subsystems.SignalManager;
import frc.robot.subsystems.drivetrain.SwerveSysId;

import au.grapplerobotics.CanBridge;

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
  private final Elevator m_elevator;
  private final Arm m_arm;
  private final EndEffector m_endEffector;
  private final RAROdometry m_odometry;
  private final Intakes m_intakes;
  private final Hopper m_hopper;
  private final TaskScheduler m_taskScheduler;
  private final DriverController m_driverController;

  private final AutoRunner m_autoRunner;
  private final AutoChooser m_autoChooser;
  private Task m_currentTask;
  private final OperatorController m_operatorController;
  private final PoseAligner m_poseAligner;
  private final SignalManager m_signalManager = SignalManager.getInstance();
  private final VirtualRobotController m_virtualRobotController;
  private final GenericHID m_sysIdController;

  private final SwerveSysId m_swerveSysId;

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
    m_arm = Arm.getInstance();
    m_elevator = Elevator.getInstance();
    m_endEffector = EndEffector.getInstance();
    m_poseAligner = PoseAligner.getInstance();
    m_intakes = Intakes.getInstance();
    m_hopper = Hopper.getInstance();
    m_taskScheduler = TaskScheduler.getInstance();

    CanBridge.runTCP();

    m_driverController = new DriverController(0, true, true, 0.5);
    m_operatorController = new OperatorController(1, true, true, 0.5);
    m_virtualRobotController = new VirtualRobotController(2);
    m_sysIdController = new GenericHID(3);

    m_subsystems.add(m_poseAligner);
    m_subsystems.add(m_swerve);
    m_subsystems.add(m_odometry);
    m_subsystems.add(m_arm);
    m_subsystems.add(m_elevator);
    m_subsystems.add(m_endEffector);
    m_subsystems.add(m_intakes);
    m_subsystems.add(m_hopper);
    m_subsystems.add(m_taskScheduler);
    
    m_swerveSysId = new SwerveSysId(m_swerve.getSwerveModules(), "SwerveSysId");
  }

  @Override
  public void robotInit() {
    new RobotTelemetry();

    // Initialize on-board logging
    DataLogManager.start();
    RobotTelemetry
        .print(String.format("Deployed version: GIT_COMMIT=%s, BUILD_DATE=%s, DIRTY=%d", BuildConstants.GIT_SHA,
            BuildConstants.BUILD_DATE, BuildConstants.DIRTY));
    RobotTelemetry.print(String.format("Branch: %s", BuildConstants.GIT_BRANCH));
    RobotTelemetry.print("Logging Initialized. Fard.");

    m_signalManager.finalizeAll();
  }

  @Override
  public void robotPeriodic() {
    if (this.isTestEnabled()) {
      CommandScheduler.getInstance().run();
    } else {
      m_virtualRobotController.updatePose();

      m_subsystems.forEach(subsystem -> subsystem.periodic());
      m_subsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
      m_subsystems.forEach(subsystem -> subsystem.writeToLog());

      m_signalManager.refresh();
    }

    if (m_operatorController.getWantsResetElevator()) {
      m_elevator.reset();
    }
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
    m_hopper.on();
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

    Pose2d targetPose = m_poseAligner.getAndCalculateTargetPose(m_virtualRobotController.getCurrentPose());
    ASPoseHelper.addPose("VirtualRobot/target", targetPose);

    Pose2d currentPose = m_odometry.getPose();

    Pose2d desiredPose = m_poseAligner.getAndCalculateTargetPose(currentPose);
    ASPoseHelper.addPose("VirtualRobot/target", desiredPose);
    if (m_driverController.getWantsAutoPosition()) {
      m_swerve.drive(xSpeed, ySpeed, rot, true, currentPose, desiredPose);
    } else {
      m_swerve.drive(xSpeed, ySpeed, rot, true);
    }

    if (m_operatorController.getWantsLeftIntakeGround()) {
      m_intakes.setIntakeState(IntakeVariant.LEFT, IntakeState.INTAKE);
    }

    if (m_operatorController.getWantsLeftIntakeStow()) {
      m_intakes.setIntakeState(IntakeVariant.LEFT, IntakeState.NONE);
    }

    if (m_operatorController.getWantsRightIntakeGround()) {
      m_intakes.setIntakeState(IntakeVariant.RIGHT, IntakeState.INTAKE);
    }

    if (m_operatorController.getWantsRightIntakeStow()) {
      m_intakes.setIntakeState(IntakeVariant.RIGHT, IntakeState.NONE);
    }

    if (m_operatorController.getWantsIntakeEject()) {
      m_intakes.setIntakeState(IntakeVariant.LEFT, IntakeState.EJECT);
      m_intakes.setIntakeState(IntakeVariant.RIGHT, IntakeState.EJECT);
    } 
    
    if (m_operatorController.getWantsIntakeStopEjecting()) {
      m_intakes.setIntakeState(IntakeVariant.LEFT, IntakeState.NONE);
      m_intakes.setIntakeState(IntakeVariant.RIGHT, IntakeState.NONE);
    }

    if (m_driverController.getWantsResetOdometry()) {
      m_odometry.reset();
    }

    if (m_operatorController.getWantsGoToStow()) {
      m_elevator.setState(ElevatorState.STOW);
      m_arm.setArmState(ArmState.STOW);
    } else if (m_operatorController.getWantsGoToL1()) {
      m_elevator.setState(ElevatorState.L1);
      m_arm.setArmState(ArmState.STOW);
    } else if (m_operatorController.getWantsGoToL2()) {
      m_elevator.setState(ElevatorState.L2);
      m_arm.setArmState(ArmState.STOW);
    } else if (m_operatorController.getWantsGoToL3()) {
      m_elevator.setState(ElevatorState.L3);
      m_arm.setArmState(ArmState.STOW);
    } else if (m_operatorController.getWantsGoToL4()) {
      m_elevator.setState(ElevatorState.L4);
      m_arm.setArmState(ArmState.EXTEND);
    } 

    if (m_operatorController.getWantsScore()) {
      if (m_elevator.getTargetState() == ElevatorState.L1) {
        m_endEffector.setState(EndEffectorState.SCORE_TROUGH);
      } else {
        m_endEffector.setState(EndEffectorState.SCORE_BRANCHES);
      }
    } else {
      m_endEffector.setState(EndEffectorState.OFF);
    }

    if (m_driverController.getWantsTest()) {
      m_taskScheduler.scheduleTask(new ParallelTask(
        new ElevatorTask(ElevatorState.L4),
        new ArmTask(ArmState.EXTEND)
      ));
      m_taskScheduler.scheduleTask(new EndEffectorTask(EndEffectorState.SCORE_BRANCHES));
    }
  }

  @Override
  public void disabledInit() {
    if (DriverStation.getMatchType() == MatchType.None) {
      m_autoRunner.initialize();
    }
    m_hopper.stop();
  }

  @Override
  public void disabledPeriodic() {
    m_odometry.setAllianceGyroAngleAdjustment();

    if (m_operatorController.getWantsResetElevator()) {
      m_elevator.reset();
    }

    // SCARY
    DriverStation.silenceJoystickConnectionWarning(DriverStation.getMatchType() == DriverStation.MatchType.None);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    if (m_sysIdController.getRawButtonPressed(FilteredController.Button.A)) {
      m_swerveSysId.sysIdDriveQuasistatic(SysIdRoutine.Direction.kForward).schedule();
    } else if (m_sysIdController.getRawButtonPressed(FilteredController.Button.B)) {
      m_swerveSysId.sysIdDriveQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
    } else if (m_sysIdController.getRawButtonPressed(FilteredController.Button.X)) {
      m_swerveSysId.sysIdDriveDynamic(SysIdRoutine.Direction.kForward).schedule();
    } else if (m_sysIdController.getRawButtonPressed(FilteredController.Button.Y)) {
      m_swerveSysId.sysIdDriveDynamic(SysIdRoutine.Direction.kReverse).schedule();
    } else if (m_sysIdController.getRawButtonPressed(FilteredController.Button.START)) {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
