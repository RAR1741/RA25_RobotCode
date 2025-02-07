package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;

// #Thanks868 <3
// https://github.com/frc868/houndutil/blob/main/src/main/java/com/techhounds/houndutil/houndlog/SignalManager.java

/**
 * Manages CTRE status signals, used to reduce performance overhead of reading
 * status signals from multiple devices.
 */
public class SignalManager {
  private static SignalManager m_signalManager;

  private final ArrayList<BaseStatusSignal> m_statusesList;
  private BaseStatusSignal[] m_statuses;

  private SignalManager() {
    m_statusesList = new ArrayList<>();
  }

  public static SignalManager getInstance() {
    if(m_signalManager == null) {
      m_signalManager = new SignalManager();
    }
    return m_signalManager;
  }

  /**
   * Registers status signals to be updated by the SignalManager.
   * 
   * @param statuses the status signals to register
   */
  public void register(BaseStatusSignal... statuses) {
    for (BaseStatusSignal status : statuses) {
      m_statusesList.add(status);
    }
  }

  /**
   * Finalizes all registered status signals. This must be called after all status
   * signals have been registered.
   */
  public void finalizeAll() {
    m_statuses = new BaseStatusSignal[m_statusesList.size()];

    m_statusesList.toArray(m_statuses);

    m_statusesList.clear();
  }

  /**
   * Refreshes all registered status signals. This should be called periodically.
   */
  public void refresh() {
    if (m_statuses.length > 0)
      BaseStatusSignal.waitForAll(0, m_statuses);
  }
}