package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;

import com.revrobotics.spark.config.SparkFlexConfig;

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

    public static EndEffector getInstance() {
        if(m_instance == null){
            m_instance = new EndEffector();
        }
        return m_instance;
    }

    private EndEffector() {
        super("EndEffector");

        m_periodicIO = new PeriodicIO();

        m_leftMotor = new RARSparkMax(RobotConstants.robotConfig.EndEffector.k_leftMotorId, MotorType.kBrushless);
        m_rightMotor = new RARSparkMax(RobotConstants.robotConfig.EndEffector.k_rightMotorId, MotorType.kBrushless);

        SparkBaseConfig shooterConfig = new SparkFlexConfig().idleMode(IdleMode.kCoast);

        shooterConfig.closedLoop
            .pidf(RobotConstants.robotConfig.EndEffector.k_P, 
                RobotConstants.robotConfig.EndEffector.k_I, 
                RobotConstants.robotConfig.EndEffector.k_D, 
                RobotConstants.robotConfig.EndEffector.k_FF)
            .minOutput(RobotConstants.robotConfig.EndEffector.k_minOutput)
            .maxOutput(RobotConstants.robotConfig.EndEffector.k_maxOutput);

        SparkBaseConfig leftShooterConfig = new SparkFlexConfig()
            .apply(shooterConfig)
            .inverted(true);

        m_leftMotor.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        SCORE_BRANCHES,
        SCORE_TROUGH
    }

    public void setState(EndEffectorState state) {
        m_periodicIO.state = state;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void periodic() {
        switch (m_periodicIO.state) {
            case OFF:
                m_periodicIO.leftSpeed = 0.0;
                m_periodicIO.rightSpeed = 0.0;
                break;
            case INDEX:
                m_periodicIO.leftSpeed = RobotConstants.robotConfig.EndEffector.k_indexSpeed;
                m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndEffector.k_indexSpeed;
                break;
            case SCORE_BRANCHES:
                m_periodicIO.leftSpeed = RobotConstants.robotConfig.EndEffector.k_branchesSpeed;
                m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndEffector.k_branchesSpeed;
                break;
            case SCORE_TROUGH:
                m_periodicIO.leftSpeed = RobotConstants.robotConfig.EndEffector.k_troughSpeed;
                m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndEffector.k_branchesSpeed;
                break;
            default:
                break;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        double limitedRightSpeed = m_rightSpeedLimiter.calculate(m_periodicIO.rightSpeed);
        double limitedLeftSpeed = m_leftSpeedLimiter.calculate(m_periodicIO.rightSpeed);
        m_leftMotor.getClosedLoopController().setReference(limitedLeftSpeed, ControlType.kVelocity);
        m_rightMotor.getClosedLoopController().setReference(limitedRightSpeed, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }
    
}
