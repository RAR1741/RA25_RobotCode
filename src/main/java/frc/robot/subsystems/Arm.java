package frc.robot.subsystems;

public class Arm extends Subsystem{
    private static Arm m_arm = null;

    private Arm() {
        super("Arm");
    }

    public static Arm getInstance(){
        if (m_arm == null) {
            m_arm = new Arm();
        }
        return m_arm;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'writePeriodicOutputs'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

}
