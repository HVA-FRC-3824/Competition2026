package frc.robot.lib;

public abstract class Module<SubsystemInputs, SubsystemOutputs extends Logged>
{
    protected SubsystemInputs  m_inputs;
    protected SubsystemOutputs m_outputs;

    // Handle logic between subsystems inputs and hardware inputs
    abstract public void updateHardwareInputs();

    // Update logging struct (m_outputs)
    abstract public void updateOutputs();

    // Update hardware inputs and subsystem outputs
    public void modulePeriodic()
    {
        updateHardwareInputs();
        updateOutputs();
        log();
    }
    
    public void setInputs(SubsystemInputs desired)
    {
        m_inputs = desired;
    }

    public SubsystemOutputs getOutputs()
    {
        return m_outputs;
    }

    // This should be fine and you shouldn't need to override this
    private void log()
    {
        m_outputs.log();
    }
}