package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotConstants;

public class Hopper extends Subsystem {
  private static Hopper m_instance = null;
  private final SparkMax m_hopperMotor;
  private static PeriodicIO m_periodicIO;

  private int m_stuckCounter = 0;
  private int m_stuckMax = 10;

  private double m_unstuckTime = 0.3;
  private Timer m_unstuckTimer = new Timer();

  public static class PeriodicIO {
    HopperState state = HopperState.OFF;
  }

  private Hopper() {
    super("Hopper");

    // create the motor
    m_hopperMotor = new SparkMax(RobotConstants.robotConfig.Hopper.k_hopperMotorId, MotorType.kBrushless);

    // create config
    SparkMaxConfig config = new SparkMaxConfig();

    // add config
    config
        .smartCurrentLimit(RobotConstants.robotConfig.Hopper.k_maxCurrent)
        .inverted(false);

    m_hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_periodicIO = new PeriodicIO();
  }

  public static Hopper getInstance() {
    if (m_instance == null) {
      m_instance = new Hopper();
    }
    return m_instance;
  }

  public void on() {
    setState(HopperState.INDEX);
  }

  public void off() {
    setState(HopperState.OFF);
  }

  public void forward() {
    setState(HopperState.INDEX);
  }

  public void reverse() {
    setState(HopperState.REVERSE);
  }

  public enum HopperState {
    OFF,
    INDEX,
    REVERSE,
    STUCK
  }

  public void setState(HopperState state) {
    // Don't allow overriding of the STUCK state
    // while the timer is still running
    if (m_periodicIO.state != HopperState.STUCK) {
      m_periodicIO.state = state;
    } else if (m_unstuckTimer.get() > m_unstuckTime) {
      m_periodicIO.state = state;
    }
  }

  @Override
  public void reset() {
    off();
  }

  @Override
  public void periodic() {
    if (getCurrent() > 15.0) {
      m_stuckCounter++;

      if (m_stuckCounter > m_stuckMax) {
        setState(HopperState.STUCK);
        m_unstuckTimer.reset();
        m_unstuckTimer.start();
      }
    } else {
      m_stuckCounter = 0;
    }

    if (m_periodicIO.state == HopperState.STUCK && m_unstuckTimer.get() > m_unstuckTime) {
      setState(HopperState.INDEX);
    }
  }

  @Override
  public void writePeriodicOutputs() {
    double speed = RobotConstants.robotConfig.Hopper.k_hopperSpeed;

    if (m_periodicIO.state == HopperState.REVERSE || m_periodicIO.state == HopperState.STUCK) {
      speed *= -0.4;
    }
    if (isHopperOn()) {
      m_hopperMotor.set(speed);
    } else {
      m_hopperMotor.set(0.0);
    }
  }

  @Override
  public void stop() {
    off();
  }

  @AutoLogOutput(key = "Hopper/Velocity")
  public double getMotorVelocityRPM() {
    return m_hopperMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Hopper/IsOn")
  public boolean isHopperOn() {
    return m_periodicIO.state != HopperState.OFF;
  }

  @AutoLogOutput(key = "Hopper/State")
  public HopperState getHopperState() {
    return m_periodicIO.state;
  }

  @AutoLogOutput(key = "Hopper/amps")
  public double getCurrent() {
    return m_hopperMotor.getOutputCurrent();
  }
}
