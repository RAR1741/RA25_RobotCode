
package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Helpers;
import frc.robot.constants.RobotConstants;

public class Elevator extends Subsystem {
  private static Elevator m_instance;
  private PeriodicIO m_periodicIO;

  // private LaserCanHandler m_laserCan;

  // private static final double k_pivotCLRampRate = 0.5;
  // private static final double k_CLRampRate = 0.5;

  public static Elevator getInstance() {
    if (m_instance == null) {
      m_instance = new Elevator();
    }
    return m_instance;
  }

  private SparkMax m_leftMotor;
  private RelativeEncoder m_leftEncoder;
  private SparkClosedLoopController m_leftPIDController;

  private SparkMax m_rightMotor;

  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_currentState = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_goalState = new TrapezoidProfile.State();
  private double m_previousUpdateTime = Timer.getFPGATimestamp();

  private Elevator() {
    super("Elevator");

    m_periodicIO = new PeriodicIO();
    // m_laserCan = LaserCanHandler.getInstance();

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.closedLoop
        .pid(RobotConstants.robotConfig.Elevator.k_P,
            RobotConstants.robotConfig.Elevator.k_I,
            RobotConstants.robotConfig.Elevator.k_D)
        .iZone(RobotConstants.robotConfig.Elevator.k_IZone);

    elevatorConfig.smartCurrentLimit(RobotConstants.robotConfig.Elevator.k_maxCurrent);

    elevatorConfig.idleMode(IdleMode.kBrake);

    // LEFT ELEVATOR MOTOR
    m_leftMotor = new SparkMax(
        RobotConstants.robotConfig.Elevator.k_elevatorLeftMotorId,
        MotorType.kBrushless);
    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftPIDController = m_leftMotor.getClosedLoopController();

    m_leftMotor.configure(
        elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // RIGHT ELEVATOR MOTOR
    m_rightMotor = new SparkMax(
        RobotConstants.robotConfig.Elevator.k_elevatorRightMotorId,
        MotorType.kBrushless);
    m_rightMotor.configure(
        elevatorConfig.follow(m_leftMotor, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            RobotConstants.robotConfig.Elevator.k_maxVelocity,
            RobotConstants.robotConfig.Elevator.k_maxAcceleration));
  }

  public enum ElevatorState {
    STOW,
    L1,
    L2,
    L3,
    L4
  }

  private static class PeriodicIO {
    ElevatorState target_state = ElevatorState.STOW;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    double currentTime = Timer.getFPGATimestamp();
    double deltaTime = currentTime - m_previousUpdateTime;
    m_previousUpdateTime = currentTime;

    // Update goal
    m_goalState.position = getElevatorTarget();

    // Calculate new state
    m_currentState = m_profile.calculate(deltaTime, m_currentState, m_goalState);

    // Set PID controller to new state
    m_leftPIDController.setReference(
        m_currentState.position,
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        RobotConstants.robotConfig.Elevator.k_FF,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    m_leftPIDController.setReference(0.0, SparkBase.ControlType.kVoltage);
  }

  @Override
  public void reset() {
    if (getTargetState() == ElevatorState.STOW) {
      m_leftEncoder.setPosition(0.0);
    }
  }

  public void setState(ElevatorState state) {
    // if the LaserCAN cannot see any coral, we can safely assume that the elevator
    // is free to move

    // if (!m_laserCan.getEntranceSeesCoral()) { TODO Add LaserCan to end effector and replace this line with that call
    m_periodicIO.target_state = state;
    // }
  }

  @AutoLogOutput(key = "Elevator/IsAtState") 
  public boolean getIsAtState() {
    if(RobotBase.isSimulation()) {
      return true;
    }
    
    double currentPos = getCurrentPosition();
    double targetPos = getElevatorTarget();
    double allowedError = RobotConstants.robotConfig.Elevator.k_allowedError;

    return Math.abs(currentPos - targetPos) < allowedError;
  }

  @AutoLogOutput(key = "Elevator/Position/Current")
  private double getCurrentPosition() {
    return m_leftEncoder.getPosition();
  }

  @AutoLogOutput(key = "Elevator/ElevatorStateOrdinal")
  public int getElevatorStateOrdinal() {
    return m_periodicIO.target_state.ordinal();
  }

  @AutoLogOutput(key = "Elevator/ElevatorState")
  public ElevatorState getTargetState() {
    return m_periodicIO.target_state;
  }

  @AutoLogOutput(key = "Elevator/Position/Target")
  private double getElevatorTarget() {
    switch (m_periodicIO.target_state) {
      case STOW -> {
        return RobotConstants.robotConfig.Elevator.k_stowHeight;
      }
      case L1 -> {
        return RobotConstants.robotConfig.Elevator.k_L1Height;
      }
      case L2 -> {
        return RobotConstants.robotConfig.Elevator.k_L2Height;
      }
      case L3 -> {
        return RobotConstants.robotConfig.Elevator.k_L3Height;
      }
      case L4 -> {
        return RobotConstants.robotConfig.Elevator.k_L4Height;
      }
      default -> {
        return RobotConstants.robotConfig.Elevator.k_stowHeight;
      }
    }
  }

  @AutoLogOutput(key = "Elevator/Velocity/Current")
  private double getElevatorVelocity() {
    return m_leftEncoder.getVelocity();
  }

  @AutoLogOutput(key = "Elevator/Position/Setpoint")
  private double getElevatorPosSetpoint() {
    return m_currentState.position;
  }

  @AutoLogOutput(key = "Elevator/Velocity/Setpoint")
  private double getElevatorVelSetpoint() {
    return m_currentState.velocity;
  }

  @AutoLogOutput(key = "Elevator/Current/Left")
  private double getElevatorCurrentLeft() {
    return m_leftMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "Elevator/Current/Right")
  private double getElevatorCurrentRight() {
    return m_rightMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "Elevator/Output/Left")
  private double getElevatorOutputLeft() {
    return m_leftMotor.getAppliedOutput();
  }

  @AutoLogOutput(key = "Elevator/Output/Right")
  private double getElevatorOutputRight() {
    return m_rightMotor.getAppliedOutput();
  }

  @AutoLogOutput(key = "Elevator/Voltage/Left")
  private double getLeftVoltage() {
    return Helpers.getVoltage(m_leftMotor);
  }

  @AutoLogOutput(key = "Elevator/Voltage/Right")
  private double getRightVoltage() {
    return Helpers.getVoltage(m_rightMotor);
  }
}
