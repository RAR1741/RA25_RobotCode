package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.LaserCanHandler;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;

public class EndEffector extends Subsystem {
  private static EndEffector m_instance;

  private PeriodicIO m_periodicIO;

  SparkMax m_leftMotor;
  SparkMax m_rightMotor;

  private LaserCanHandler m_laserCan;
  private Arm m_arm;
  private Elevator m_elevator;

  public static EndEffector getInstance() {
    if (m_instance == null) {
      m_instance = new EndEffector();
    }
    return m_instance;
  }

  private EndEffector() {
    super("EndEffector");

    m_periodicIO = new PeriodicIO();

    m_leftMotor = new SparkMax(RobotConstants.robotConfig.EndEffector.k_leftMotorId, MotorType.kBrushless);
    m_rightMotor = new SparkMax(RobotConstants.robotConfig.EndEffector.k_rightMotorId, MotorType.kBrushless);

    m_laserCan = LaserCanHandler.getInstance();
    m_arm = Arm.getInstance();
    m_elevator = Elevator.getInstance();

    SparkBaseConfig endEffectorConfig = new SparkFlexConfig().idleMode(IdleMode.kBrake);

    m_rightMotor.configure(
        endEffectorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_leftMotor.configure(
        endEffectorConfig.inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private static class PeriodicIO {
    EndEffectorState state = EndEffectorState.INDEXED;
  }

  public enum EndEffectorState {
    OFF,
    FORWARD_INDEX_FAST,
    FORWARD_INDEX_SLOW,
    REVERSE_INDEX,
    SCORE_BRANCHES,
    SCORE_TROUGH,
    INDEXED
  }

  public void setState(EndEffectorState state) {
    m_periodicIO.state = state;
  }

  private void off() {
    setState(EndEffectorState.OFF);
  }

  private void indexFast() {
    setState(EndEffectorState.FORWARD_INDEX_FAST);
  }

  private void branches() {
    setState(EndEffectorState.SCORE_BRANCHES);
  }

  private void trough() {
    setState(EndEffectorState.SCORE_TROUGH);
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void periodic() {
    checkAutoTasks();
  }

  @Override
  public void writePeriodicOutputs() {
    double[] speeds = getIntakeSpeeds();

    m_leftMotor.set(speeds[0]);
    m_rightMotor.set(speeds[1]);
  }

  @AutoLogOutput(key = "EndEffector/IsSafeToScore")
  public boolean isSafeToScore() {
    return m_periodicIO.state == EndEffectorState.INDEXED;
  }

  @AutoLogOutput(key = "EndEffector/State")
  public EndEffectorState getEndEffectorState() {
    return m_periodicIO.state;
  }

  @AutoLogOutput(key = "EndEffector/Position/Target")
  private double[] getIntakeSpeeds() {
    switch (m_periodicIO.state) {
      case OFF -> {
        return RobotConstants.robotConfig.EndEffector.k_stopSpeeds;
      }

      case FORWARD_INDEX_SLOW -> {
        return RobotConstants.robotConfig.EndEffector.k_forwardIndexSlowSpeeds;
      }

      case FORWARD_INDEX_FAST -> {
        return RobotConstants.robotConfig.EndEffector.k_forwardIndexFastSpeeds;
      }

      case REVERSE_INDEX -> {
        return RobotConstants.robotConfig.EndEffector.k_reverseIndexSpeeds;
      }

      case SCORE_BRANCHES -> {
        if (m_arm.getArmState() == ArmState.EXTEND || m_elevator.getTargetState() == ElevatorState.L4) {
          return new double[] { -RobotConstants.robotConfig.EndEffector.k_branchSpeeds[0],
              -RobotConstants.robotConfig.EndEffector.k_branchSpeeds[1] };
        }
        return RobotConstants.robotConfig.EndEffector.k_branchSpeeds;
      }

      case SCORE_TROUGH -> {
        return RobotConstants.robotConfig.EndEffector.k_troughSpeeds;
      }

      default -> {
        return RobotConstants.robotConfig.EndEffector.k_stopSpeeds;
      }
    }
  }

  @Override
  public void stop() {
    // we might want more here later, but this is enough for testing
    off();
  }

  private void checkAutoTasks() {
    switch (m_periodicIO.state) {
      case FORWARD_INDEX_FAST -> {
        if (m_laserCan.getExitSeesCoral()) {
          setState(EndEffectorState.FORWARD_INDEX_SLOW);
        }
      }

      case FORWARD_INDEX_SLOW -> {
        if (!m_laserCan.getEntranceSeesCoral()) {
          setState(EndEffectorState.REVERSE_INDEX);
        }
      }

      case REVERSE_INDEX -> {
        if (m_laserCan.getEntranceSeesCoral()) {
          setState(EndEffectorState.INDEXED);
        }
      }

      case INDEXED -> {
        if (!m_laserCan.getExitSeesCoral()) {
          setState(EndEffectorState.OFF);
        }
      }

      case OFF -> {
        // if (!(m_laserCan.getEntranceSeesCoral() || m_laserCan.getIndexSeesCoral())
        // && m_laserCan.getExitSeesCoral()) {
        // reverse();
        // } else
        if (m_laserCan.getEntranceSeesCoral()) {
          indexFast();
        }
      }

      case SCORE_BRANCHES -> {
        if (!m_laserCan.getExitSeesCoral()) {
          off();
        } else {
          branches();
        }
      }

      case SCORE_TROUGH -> {
        if (!m_laserCan.getExitSeesCoral()) {
          off();
        } else {
          trough();
        }
      }

      default -> {
      }
    }
  }
}
