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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotTelemetry;
import frc.robot.constants.RobotConstants;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Elevator extends Subsystem {
  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Elevator m_instance;
  private PeriodicIO m_periodicIO;

  // private static final double kPivotCLRampRate = 0.5;
  // private static final double kCLRampRate = 0.5;

  public static Elevator getInstance() {
    if (m_instance == null) {
      m_instance = new Elevator();
    }
    return m_instance;
  }

  private SimulatableCANSparkMax m_leftMotor;
  private RelativeEncoder m_leftEncoder;
  private SparkClosedLoopController m_leftPIDController;

  private SimulatableCANSparkMax m_rightMotor;
  private RelativeEncoder m_rightEncoder;
  private SparkClosedLoopController m_rightPIDController;

  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_currentState = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_goalState = new TrapezoidProfile.State();
  private double m_previousUpdateTime = Timer.getFPGATimestamp();

  private Elevator() {
    super("Elevator");

    m_periodicIO = new PeriodicIO();

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.closedLoop
        .pid(RobotConstants.robotConfig.Elevator.k_P, RobotConstants.robotConfig.Elevator.k_I, RobotConstants.robotConfig.Elevator.k_D)
        .iZone(RobotConstants.robotConfig.Elevator.k_IZone)
        .minOutput(RobotConstants.robotConfig.Elevator.k_maxPowerDown)
        .maxOutput(RobotConstants.robotConfig.Elevator.k_maxPowerUp);

    elevatorConfig.smartCurrentLimit(RobotConstants.robotConfig.Elevator.k_maxCurrent);

    elevatorConfig.idleMode(IdleMode.kBrake);

    // LEFT ELEVATOR MOTOR
    m_leftMotor = new SimulatableCANSparkMax(RobotConstants.robotConfig.Elevator.k_elevatorLeftMotorId, MotorType.kBrushless);
    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftPIDController = m_leftMotor.getClosedLoopController();
    m_leftMotor.configure(
        elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // RIGHT ELEVATOR MOTOR
    m_rightMotor = new SimulatableCANSparkMax(RobotConstants.robotConfig.Elevator.k_elevatorRightMotorId, MotorType.kBrushless);
    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightPIDController = m_rightMotor.getClosedLoopController();
    m_rightMotor.configure(
        elevatorConfig.follow(m_leftMotor),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            RobotConstants.robotConfig.Elevator.k_maxVelocity,
            RobotConstants.robotConfig.Elevator.k_maxAcceleration));
  }

  public enum ElevatorState {
    NONE,
    STOW,
    L1,
    L2,
    L3,
    L4,
    // A1, // Algae L2
    // A2 // Algae L3
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    // TODO: Use this pattern to only drive slowly when we're really high up
    // if(mPivotEncoder.getPosition() > Constants.kPivotScoreCount) {
    // mPeriodicIO.is_pivot_low = true;
    // } else {
    // mPeriodicIO.is_pivot_low = false;
    // }
  }

  @Override
  public void writePeriodicOutputs() {
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - m_previousUpdateTime;
    m_previousUpdateTime = curTime;
    if (m_periodicIO.is_elevator_pos_control) {
      // Update goal
      m_goalState.position = m_periodicIO.elevator_target;

      // Calculate new state
      m_previousUpdateTime = curTime;
      m_currentState = m_profile.calculate(dt, m_currentState, m_goalState);

      // Set PID controller to new state
      m_leftPIDController.setReference(
          m_currentState.position,
          SparkBase.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          RobotConstants.robotConfig.Elevator.k_FF,
          ArbFFUnits.kVoltage);
    } else {
      m_currentState.position = m_leftEncoder.getPosition();
      m_currentState.velocity = 0;
      m_leftMotor.set(m_periodicIO.elevator_power);
    }
  }

  @Override
  public void stop() {
    m_periodicIO.is_elevator_pos_control = false;
    m_periodicIO.elevator_power = 0.0;

    m_leftMotor.set(0.0);
  }
  

  @Override
  public void reset() {
    m_leftEncoder.setPosition(0.0);
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public ElevatorState getState() {
    return m_periodicIO.state;
  }

  public void setElevatorPower(double power) {
    putNumber("setElevatorPower", power);
    m_periodicIO.is_elevator_pos_control = false;
    m_periodicIO.elevator_power = power;
  }

  public void goToElevatorStow() {
    m_periodicIO.is_elevator_pos_control = true;
    m_periodicIO.elevator_target = RobotConstants.robotConfig.Elevator.k_stowHeight;
    m_periodicIO.state = ElevatorState.STOW;
  }

  public void goToElevatorL1() {
    m_periodicIO.is_elevator_pos_control = true;
    m_periodicIO.elevator_target = RobotConstants.robotConfig.Elevator.k_L1Height;
    m_periodicIO.state = ElevatorState.L1;
  }

  public void goToElevatorL2() {
    m_periodicIO.is_elevator_pos_control = true;
    m_periodicIO.elevator_target = RobotConstants.robotConfig.Elevator.k_L2Height;
    m_periodicIO.state = ElevatorState.L2;
  }

  public void goToElevatorL3() {
    m_periodicIO.is_elevator_pos_control = true;
    m_periodicIO.elevator_target = RobotConstants.robotConfig.Elevator.k_L3Height;
    m_periodicIO.state = ElevatorState.L3;
  }

  public void goToElevatorL4() {
    m_periodicIO.is_elevator_pos_control = true;
    m_periodicIO.elevator_target = RobotConstants.robotConfig.Elevator.k_L4Height;
    m_periodicIO.state = ElevatorState.L4;
  }

  // public void goToAlgaeLow() {
  //   m_periodicIO.is_elevator_pos_control = true;
  //   m_periodicIO.elevator_target = RobotConstants.robotConfig.Elevator.kLowAlgaeHeight;
  //   m_periodicIO.state = ElevatorState.A1;
  // }

  // public void goToAlgaeHigh() {
  //   m_periodicIO.is_elevator_pos_control = true;
  //   m_periodicIO.elevator_target = RobotConstants.robotConfig.Elevator.kHighAlgaeHeight;
  //   m_periodicIO.state = ElevatorState.A2;
  // }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  @AutoLogOutput(key = "Elevator/Position/Current")
  private double getCurrentPosition() {
    return m_leftEncoder.getPosition();
  }

  @AutoLogOutput(key = "Elevator/ElevatorStateOrdinal")
  public int getElevatorStateOrdinal() {
    return m_periodicIO.state.ordinal();
  }

  @AutoLogOutput(key = "Elevator/Position/Target")
  private double getElevatorTarget() {
    return m_periodicIO.elevator_target;
  }

  @AutoLogOutput(key = "Elevator/Velocity/Current")
  private double getElevatorvelocity() {
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

}
