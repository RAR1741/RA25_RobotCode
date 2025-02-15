package frc.robot.subsystems.drivetrain;

import java.util.concurrent.locks.ReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotConstants;

/**
 * Thread enabling 250Hz odometry. Optimized from CTRE's internal swerve code.
 * 250Hz odometry reduces discretization error in the odometry loop, and
 * significantly improves odometry during high speed maneuvers.
 */
public class OdometryThread implements Runnable {
  // Testing shows 1 (minimum realtime) is sufficient for tighter odometry loops.
  // If the odometry period is far away from the desired frequency, increasing
  // this may help
  private static final int START_THREAD_PRIORITY = 1;

  private final Thread m_thread;
  private volatile boolean m_running = false;

  private final BaseStatusSignal[] m_allSignals;

  private SwerveDrive m_swerve;
  private RAROdometry m_odometry;

  private SwerveDrivePoseEstimator m_poseEstimator;
  private AHRS m_gyro;

  /** Lock used for odometry thread. */
  private final ReadWriteLock m_stateLock;

  private final MedianFilter m_peakRemover = new MedianFilter(3);
  private final LinearFilter m_lowPass = LinearFilter.movingAverage(50);

  private SwerveModule[] m_modules;

  private volatile int m_threadPriorityToSet = START_THREAD_PRIORITY;
  private final int k_updateFrequency = RobotConstants.robotConfig.Odometry.k_threadUpdateFrequency;
  private int m_lastThreadPriority = START_THREAD_PRIORITY;

  private double m_lastTime = 0;
  private double m_currentTime = 0;

  private double m_averageOdometryLoopTime = 0.0;

  private int m_successfulDaqs = 0;
  private int m_failedDaqs = 0;

  public OdometryThread(ReadWriteLock stateLock) {
    m_stateLock = stateLock;

    m_thread = new Thread(this);

    m_swerve = SwerveDrive.getInstance();
    m_modules = m_swerve.getSwerveModules();
    /*
     * Mark this thread as a "daemon" (background) thread
     * so it doesn't hold up program shutdown
     */
    m_thread.setDaemon(true);

    /* 4 signals for each module + 2 for Pigeon2 */
    m_allSignals = new BaseStatusSignal[4 * 4];
    for (int i = 0; i < 4; ++i) {
      BaseStatusSignal[] signals = m_modules[i].getSignals();
      m_allSignals[(i * 4) + 0] = signals[0];
      m_allSignals[(i * 4) + 1] = signals[1];
      m_allSignals[(i * 4) + 2] = signals[2];
      m_allSignals[(i * 4) + 3] = signals[3];
    }
  }

  /**
   * Starts the odometry thread.
   */
  public void start() {
    m_odometry = RAROdometry.getInstance();
    m_poseEstimator = m_odometry.getPoseEstimator();
    m_gyro = m_odometry.getGyro();

    m_running = true;
    m_thread.start();
  }

  /**
   * Stops the odometry thread
   */
  public void stop() {
    try {
      m_thread.join(1);
    } catch (final InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
    m_successfulDaqs = 0;
    m_failedDaqs = 0;
    m_running = false;
  }

  @Override
  public void run() {
    /* Make sure all signals update at the correct update frequency */
    BaseStatusSignal.setUpdateFrequencyForAll(k_updateFrequency, m_allSignals);
    Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

    /* Run as fast as possible, our signals will control the timing */
    while (m_running) {
      /* Synchronously wait for all signals in drivetrain */
      /* Wait up to twice the period of the update frequency */
      StatusCode status = BaseStatusSignal.waitForAll(2.0 / k_updateFrequency, m_allSignals);

      try {
        m_stateLock.writeLock().lock();

        m_lastTime = m_currentTime;
        m_currentTime = Timer.getFPGATimestamp();

        /*
         * We don't care about the peaks, as they correspond to GC events, and we want
         * the period generally low passed
         */
        m_averageOdometryLoopTime = m_lowPass.calculate(m_peakRemover.calculate(m_currentTime - m_lastTime));

        /* Get status of first element */
        if (status.isOK()) {
          m_successfulDaqs++;
        } else {
          m_failedDaqs++;
        }

        /* Keep track of previous and current pose to account for the carpet vector */
        m_poseEstimator.updateWithTime(
            m_currentTime,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_swerve.getModule(SwerveDrive.Module.FRONT_LEFT).getPosition(),
                m_swerve.getModule(SwerveDrive.Module.FRONT_RIGHT).getPosition(),
                m_swerve.getModule(SwerveDrive.Module.BACK_RIGHT).getPosition(),
                m_swerve.getModule(SwerveDrive.Module.BACK_LEFT).getPosition()
            });
      } finally {
        m_stateLock.writeLock().unlock();
      }

      /**
       * This is inherently synchronous, since lastThreadPriority
       * is only written here and threadPriorityToSet is only read here
       */
      if (m_threadPriorityToSet != m_lastThreadPriority) {
        Threads.setCurrentThreadPriority(true, m_threadPriorityToSet);
        m_lastThreadPriority = m_threadPriorityToSet;
      }
    }
  }

  public ReadWriteLock getStateLock() {
    return m_stateLock;
  }

  /**
   * Sets the DAQ thread priority to a real time priority under the specified
   * priority level
   *
   * @param priority Priority level to set the DAQ thread to.
   *                 This is a value between 0 and 99, with 99 indicating higher
   *                 priority and 0 indicating lower priority.
   */
  public void setThreadPriority(int priority) {
    m_threadPriorityToSet = priority;
  }

  // TODO: Log these outside of the odometry thread
  // @AutoLogOutput(key = "Odometry/Thread/SuccessfulDataAquisitions")
  public int getSuccessfulDaqs() {
    return m_successfulDaqs;
  }

  // @AutoLogOutput(key = "Odometry/Thread/FailedDataAquisitions")
  public int getFailedDaqs() {
    return m_failedDaqs;
  }

  // @AutoLogOutput(key = "Odometry/Thread/AverageLoopTime")
  public double getAverageOdometryLoopTime() {
    return m_averageOdometryLoopTime;
  }

  // @AutoLogOutput(key = "Odometry/Thread/UpdatesPerSecond")
  public int getUpdatesPerSecond() {
    return (int) (1.0 / getAverageOdometryLoopTime());
  }

  // @AutoLogOutput(key = "Odometry/Thread/Running")
  public boolean isRunning() {
    return m_running;
  }
}
