package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
  private SignalManager m_signalManager = SignalManager.getInstance();

  public Subsystem(String loggingKey) {
    this.loggingKey = loggingKey;
  }

  /**
   * Writes the relevant subsystem information to the log
   */
  public void writeToLog() {
  }

  /**
   * Reset sensors, PID Controllers, and any private instance variables
   */
  public abstract void reset();

  /**
   * Contains the algorithms and logic to update the values in mPeriodicIO
   * <p>
   * Do not set the outputs of the system here!!
   * <p>
   * This function is called each periodic cycle before the
   * {@link #writePeriodicOutputs()} function
   */
  public abstract void periodic();

  /**
   * <p>
   * Using the current values in mPeriodicIO, sets the outputs of this subsystem.
   * <p>
   * Examples:
   *
   * <pre>
   *m_motor.set(m_periodicIO.speed);
   *m_solenoid.set(m_periodicIO.open);
   *m_LED.setRGB(...);
   *etc...
   * </pre>
   * <p>
   * The value of mPeriodicIO variables should not be changed in this function.
   * <p>
   * This function is called each periodic cycle after the {@link #periodic()}
   * function
   */
  public abstract void writePeriodicOutputs();

  /**
   * Stops the subsystem, putting it in a state that is safe for people to touch
   * the robot.
   * <p>
   * Called once when the robot is entering the disabled state
   * {@link #disabledInit()}
   */
  public abstract void stop();

  public void registerSignal(BaseStatusSignal... statuses) {
    m_signalManager.register(statuses);
  }

  public String loggingKey = "UnknownSubsystem/";

  public void putNumber(String key, double value) {
    SmartDashboard.putNumber(loggingKey + "/" + key, value);
  }

  public void putBoolean(String key, boolean value) {
    SmartDashboard.putBoolean(loggingKey + "/" + key, value);
  }

  public void putString(String key, String value) {
    SmartDashboard.putString(loggingKey + "/" + key, value);
  }

  public void putNumberArray(String key, double[] value) {
    SmartDashboard.putNumberArray(loggingKey + "/" + key, value);
  }

  public void putBooleanArray(String key, boolean[] value) {
    SmartDashboard.putBooleanArray(loggingKey + "/" + key, value);
  }

  public void putStringArray(String key, String[] value) {
    SmartDashboard.putStringArray(loggingKey + "/" + key, value);
  }

  public void putData(String key, Sendable value) {
    SmartDashboard.putData(loggingKey + "/" + key, value);
  }
}
