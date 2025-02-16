package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Elevator.ElevatorState;

public class Hopper extends Subsystem {
  private static Hopper m_instance = null;
  private final SparkMax m_hopperMotor;
  private static PeriodicIO m_periodicIO;
  private Elevator m_elevator;

  public static class PeriodicIO {
    boolean is_hopper_on = false;
  }

  private Hopper() {
    super("Hopper");

    // create the motor
    m_hopperMotor = new SparkMax(RobotConstants.robotConfig.Hopper.k_hopperMotorId, MotorType.kBrushless);

    // create config
    SparkMaxConfig config = new SparkMaxConfig();

    // add config
    config.inverted(false);
    m_hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_elevator = Elevator.getInstance();

    m_periodicIO = new PeriodicIO();
  }

  public static Hopper getInstance() {
    if (m_instance == null) {
      m_instance = new Hopper();
    }
    return m_instance;
  }

  public void on() {
    m_periodicIO.is_hopper_on = true;
  }

  public void off() {
    m_periodicIO.is_hopper_on = false;
  }

  @Override
  public void reset() {
    off();
  }

  @Override
  public void periodic() {
    if (m_elevator.getTargetState() == ElevatorState.STOW) {
      on();
    } else {
      off();
    }
  }

  @Override
  public void writePeriodicOutputs() {
    if (isHopperOn()) {
      m_hopperMotor.set(0.1);
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
    return m_periodicIO.is_hopper_on;
  }
}