package frc.robot.lib.BT;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;

public class ActionNode extends Node
{
    public ActionNode(Runnable runner) 
    {
        m_runner = Optional.of(runner);
    }

    public ActionNode(Command runner) 
    {
        m_runnerCommand = Optional.of(runner);
    }

    @Override
    public NodeStatus update(XboxController controller)
    {
        if (m_runner.isPresent())
            m_runner.get().run();
        else
            CommandScheduler.getInstance().schedule(m_runnerCommand.get());

        return NodeStatus.Success;
    }

    // DECORATORS

    public Node and(Node next)
    {
        return new SequenceNode(this, next);
    }
    
    private Optional<Runnable> m_runner;
    private Optional<Command>  m_runnerCommand;
}
