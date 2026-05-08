package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Module<SubsystemOutputs extends Module.Logged> extends SubsystemBase {

  private boolean m_hasSetup = false;

  protected SubsystemOutputs m_outputs;

  public abstract void updateOutputs();
    
  @Override
  public void periodic() {
    
    // basically a constructor, but I don't want to add one to avoid downstream complexity
    if (!m_hasSetup) {
      SmartDashboard.putData(this);
      m_hasSetup  = true;
    }

    log();
    updateOutputs();
  }

  public SubsystemOutputs getOutputs() {
    return m_outputs;
  }

  // This should be fine and you shouldn't need to override this
  private void log() {

    m_outputs.log();
  }

  // Use Logger.recordOutput to output all members of the class
  public static interface Logged { public void log(); }
}