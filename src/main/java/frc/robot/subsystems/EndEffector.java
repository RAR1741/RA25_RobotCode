package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;

import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.LaserCanHandler;
import frc.robot.constants.RobotConstants;
import frc.robot.wrappers.RARSparkMax;

public class EndEffector extends Subsystem {
    private static EndEffector m_instance;
    
    private PeriodicIO m_periodicIO;

    RARSparkMax m_leftMotor;
    RARSparkMax m_rightMotor;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    private SlewRateLimiter m_leftSpeedLimiter = new SlewRateLimiter(1000);
    private SlewRateLimiter m_rightSpeedLimiter = new SlewRateLimiter(1000);

    private LaserCanHandler m_laserCan;

    public static EndEffector getInstance() {
        if (m_instance == null) {
            m_instance = new EndEffector();
        }
        return m_instance;
    }

    private EndEffector() {
        super("EndAffector");

        m_periodicIO = new PeriodicIO();

        m_leftMotor = new RARSparkMax(RobotConstants.robotConfig.EndEffector.k_leftMotorId, MotorType.kBrushless);
        m_rightMotor = new RARSparkMax(RobotConstants.robotConfig.EndEffector.k_rightMotorId, MotorType.kBrushless);

        m_laserCan = LaserCanHandler.getInstance();

        SparkBaseConfig endEffectorConfig = new SparkFlexConfig().idleMode(IdleMode.kCoast);

        endEffectorConfig.closedLoop
                .pidf(RobotConstants.robotConfig.EndEffector.k_P,
                        RobotConstants.robotConfig.EndEffector.k_I,
                        RobotConstants.robotConfig.EndEffector.k_D,
                        RobotConstants.robotConfig.EndEffector.k_FF)
                .minOutput(RobotConstants.robotConfig.EndEffector.k_minOutput)
                .maxOutput(RobotConstants.robotConfig.EndEffector.k_maxOutput);

        SparkBaseConfig leftShooterConfig = new SparkFlexConfig()
                .apply(endEffectorConfig)
                .inverted(true);

        m_leftMotor.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.configure(endEffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();
    }

    private static class PeriodicIO {
        double leftSpeed = 0.0;
        double rightSpeed = 0.0;

        EndEffectorState state = EndEffectorState.OFF;
    }

    public enum EndEffectorState {
        OFF,
        INDEX,
        REVERSE,
        SCORE_BRANCHES,
        SCORE_TROUGH
    }

    public void setState(EndEffectorState state) {
        m_periodicIO.state = state;
    }

    private void off() {
        setState(EndEffectorState.OFF);
        m_periodicIO.leftSpeed = 0.0;
        m_periodicIO.rightSpeed = 0.0;
    }

    private void index() {
        setState(EndEffectorState.INDEX);
        m_periodicIO.leftSpeed = RobotConstants.robotConfig.EndEffector.k_indexSpeed;
        m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndEffector.k_indexSpeed;
    }

    private void reverse() {
        setState(EndEffectorState.REVERSE);
        m_periodicIO.leftSpeed = -RobotConstants.robotConfig.EndEffector.k_indexSpeed;
        m_periodicIO.rightSpeed = -RobotConstants.robotConfig.EndEffector.k_indexSpeed;
    }

    private void branches() {
        m_periodicIO.leftSpeed = RobotConstants.robotConfig.EndEffector.k_branchesSpeed;
        m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndEffector.k_branchesSpeed;
    }

    private void trough() {
        m_periodicIO.leftSpeed = RobotConstants.robotConfig.EndEffector.k_branchesSpeed;
        m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndEffector.k_troughSpeed;
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
        double limitedRightSpeed = m_rightSpeedLimiter.calculate(m_periodicIO.rightSpeed);
        double limitedLeftSpeed = m_leftSpeedLimiter.calculate(m_periodicIO.leftSpeed);
        
        m_leftMotor.getClosedLoopController().setReference(limitedLeftSpeed, ControlType.kVelocity);
        m_rightMotor.getClosedLoopController().setReference(limitedRightSpeed, ControlType.kVelocity);
    }

    @AutoLogOutput(key = "EndEffector/State")
    public EndEffectorState getEndEffectorState() {
        return m_periodicIO.state;
    }

    @AutoLogOutput(key = "EndEffector/LeftMotorSpeed")
    public double getEndEffectorLeftMotorSpeed() {
        return m_leftEncoder.getVelocity();
    }

    @AutoLogOutput(key = "EndEffector/RightMotorSpeed")
    public double getEndEffectorRightMotorSpeed() {
        return m_rightEncoder.getVelocity();
    }

    @Override
    public void stop() {
        // we might want more here later, but this is enough for testing
        off();
    }

    private void checkAutoTasks() {
        switch(m_periodicIO.state) {
            case INDEX:
                if (!m_laserCan.getEntranceSeesCoral()) {
                    off();
                }
                break;
            case OFF:
                if (!(m_laserCan.getEntranceSeesCoral() || m_laserCan.getIndexSeesCoral()) && m_laserCan.getExitSeesCoral()) {
                    reverse();
                } else if (m_laserCan.getEntranceSeesCoral()) {
                    index();
                }
                break;
            case REVERSE:
                if (m_laserCan.getIndexSeesCoral()) {
                    off();
                }
                break;
            case SCORE_BRANCHES:
                if (!m_laserCan.getExitSeesCoral()) {
                    off();
                } else {
                    branches();
                }
                break;
            case SCORE_TROUGH:
                if (!m_laserCan.getExitSeesCoral()) {
                    off();
                } else {
                    trough();
                }
                break;
            default:
                break;
        }
    }
}