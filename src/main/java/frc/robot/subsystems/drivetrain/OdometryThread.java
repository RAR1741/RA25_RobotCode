package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.studica.frc.AHRS;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;

/**
 * Thread enabling 250Hz odometry. Optimized from CTRE's internal swerve code.
 * 250Hz odometry reduces discretization error in the odometry loop, and
 * significantly improves odometry during high speed maneuvers.
 */
public class OdometryThread {
    // Testing shows 1 (minimum realtime) is sufficient for tighter odometry loops.
    // If the odometry period is far away from the desired frequency, increasing
    // this may help
    private static final int START_THREAD_PRIORITY = 1;

    private final Thread m_thread;
    private volatile boolean m_running = false;

    private final BaseStatusSignal[] allSignals;
    private final SwerveDrive m_swerve = SwerveDrive.getInstance();
    private final RAROdometry m_odometry = RAROdometry.getInstance();
    
    private final SwerveDrivePoseEstimator m_poseEstimator = m_odometry.getPoseEstimator();
    private final AHRS m_gyro = m_odometry.getGyro();

    /** Lock used for odometry thread. */
    private final ReadWriteLock stateLock = new ReentrantReadWriteLock();

    private final MedianFilter peakRemover = new MedianFilter(3);
    private final LinearFilter lowPass = LinearFilter.movingAverage(50);
    private double lastTime = 0;
    private double currentTime = 0;

    //TODO: LOG DEEZ PLS THANKS
    private int successfulDaqs = 0;
    private int failedDaqs = 0;
    private double averageOdometryLoopTime = 0.0;

    private SwerveModule[] modules = m_swerve.getSwerveModules();
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private int lastThreadPriority = START_THREAD_PRIORITY;
    private volatile int threadPriorityToSet = START_THREAD_PRIORITY;
    private final int UPDATE_FREQUENCY = 250;

    public OdometryThread() {
        m_thread = new Thread(this::run);
        /*
         * Mark this thread as a "daemon" (background) thread
         * so it doesn't hold up program shutdown
         */
        m_thread.setDaemon(true);

        /* 4 signals for each module + 2 for Pigeon2 */

        allSignals = new BaseStatusSignal[4 * 4];
        for (int i = 0; i < 4; ++i) {
            BaseStatusSignal[] signals = modules[i].getSignals();
            allSignals[(i * 4) + 0] = signals[0];
            allSignals[(i * 4) + 1] = signals[1];
            allSignals[(i * 4) + 2] = signals[2];
            allSignals[(i * 4) + 3] = signals[3];
        }
        // allSignals[allSignals.length - 2] = m_gyro.getYaw();
        // allSignals[allSignals.length - 1] = m_gyro.getRate();
    }

    /**
     * Starts the odometry thread.
     */
    public void start() {
        m_running = true;
        m_thread.start();
    }

    /**
     * Stops the odometry thread.
     */
    public void stop() {
        stop(0);
    }

    /**
     * Stops the odometry thread with a timeout.
     *
     * @param millis The time to wait in milliseconds
     */
    public void stop(long millis) {
        m_running = false;
        try {
            m_thread.join(millis);
        } catch (final InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public void run() {
        /* Make sure all signals update at the correct update frequency */
        BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQUENCY, allSignals);
        Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

        /* Run as fast as possible, our signals will control the timing */
        while (m_running) {
            /* Synchronously wait for all signals in drivetrain */
            /* Wait up to twice the period of the update frequency */
            StatusCode status;
            status = BaseStatusSignal.waitForAll(2.0 / UPDATE_FREQUENCY, allSignals);

            try {
                stateLock.writeLock().lock();

                lastTime = currentTime;
                currentTime = Timer.getFPGATimestamp();
                /*
                 * We don't care about the peaks, as they correspond to GC events, and we want
                 * the period generally low passed
                 */
                averageOdometryLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

                /* Get status of first element */
                if (status.isOK()) {
                    successfulDaqs++;
                } else {
                    failedDaqs++;
                }

                /* Now update odometry */
                /* Keep track of the change in azimuth rotations */
                for (int i = 0; i < 4; ++i) {
                    modulePositions[i] = modules[i].getPosition();
                }
                double yawDegrees = m_gyro.getYaw(); //TODO: Check if this is right 😈

                /* Keep track of previous and current pose to account for the carpet vector */
                m_poseEstimator.update(Rotation2d.fromDegrees(yawDegrees), modulePositions);
                // if (RobotBase.isSimulation()) {
                //     simOdometry.update(m_odometry.getRotation2d(), m_odometry.getModulePositions());
                // }
            } finally {
                stateLock.writeLock().unlock();
            }

            /**
             * This is inherently synchronous, since lastThreadPriority
             * is only written here and threadPriorityToSet is only read here
             */
            if (threadPriorityToSet != lastThreadPriority) {
                Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                lastThreadPriority = threadPriorityToSet;
            }
        }
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
        threadPriorityToSet = priority;
    }
}