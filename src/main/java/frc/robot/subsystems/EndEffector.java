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
        // if (!m_laserCan.getIndexSeesCoral()) {
        //     m_periodicIO.state = state;
        // }

        m_periodicIO.state = state;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void periodic() {
        updateState();

        if (m_periodicIO.state == EndEffectorState.INDEX || m_periodicIO.state == EndEffectorState.REVERSE) {
            if (m_laserCan.getEntranceSeesCoral()) {
                // even if we're in the reverse state, this theoretically should undo those negatives speeds
                setIndexSpeeds(false);
            } else if (!m_laserCan.getIndexSeesCoral()) {
                // neither the entrance or inner LaserCAN can see a coral, so reverse and try to get it back
                setState(EndEffectorState.REVERSE);
                setIndexSpeeds(true); // invert the speeds for reversing (its just indexing but not)
            } else {
                stop();
            }
        } else {
            if (m_laserCan.getExitSeesCoral()) {
                if (m_periodicIO.state == EndEffectorState.SCORE_BRANCHES) {
                    setBranchesScoringSpeeds();
                } else if (m_periodicIO.state == EndEffectorState.SCORE_TROUGH) {
                    setTroughScoringSpeeds();
                } else {
                    stop(); // if we're just trying to move the piece then stop moving
                }
            } else {
                stop();
            }
        }
    }

    private void updateState() {
        if (m_laserCan.getEntranceSeesCoral()) {
            setState(EndEffectorState.INDEX);
        // } else if (m_periodicIO.state == EndEffectorState.INDEX) {
        //     stop();
        } else if (m_periodicIO.state == EndEffectorState.OFF) {
            setZeroSpeeds();
        }
    }

    @Override
    public void writePeriodicOutputs() {
        double limitedRightSpeed = m_rightSpeedLimiter.calculate(m_periodicIO.rightSpeed);
        double limitedLeftSpeed = m_leftSpeedLimiter.calculate(m_periodicIO.leftSpeed);
        
        m_leftMotor.getClosedLoopController().setReference(limitedLeftSpeed, ControlType.kVelocity);
        m_rightMotor.getClosedLoopController().setReference(limitedRightSpeed, ControlType.kVelocity);
    }

    private void setIndexSpeeds(boolean invertSpeeds) {
        m_periodicIO.leftSpeed = m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndEffector.k_indexSpeed;

        // i dont like this but i also dont want to make the world's weirdest ternary
        if (invertSpeeds) {
            m_periodicIO.leftSpeed *= -1;
            m_periodicIO.rightSpeed *= -1;
        }
    }

    private void setBranchesScoringSpeeds() {
        m_periodicIO.leftSpeed = m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndEffector.k_branchesSpeed;
    }

    private void setTroughScoringSpeeds() {
        m_periodicIO.leftSpeed = m_periodicIO.rightSpeed = RobotConstants.robotConfig.EndEffector.k_troughSpeed;
    }

    private void setZeroSpeeds() {
        m_periodicIO.leftSpeed = 0.0;
        m_periodicIO.rightSpeed = 0.0;
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
        setState(EndEffectorState.OFF);
        setZeroSpeeds();
    }
}