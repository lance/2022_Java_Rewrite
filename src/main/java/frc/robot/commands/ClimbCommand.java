package frc.robot.commands;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
public class ClimbCommand extends CommandBase{
    private final Climber m_subsystem;
    private final DoubleSupplier xRotate;
    private final DoubleSupplier zClimb;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
    */

    public ClimbCommand(Climber subsystem, DoubleSupplier xAxis, DoubleSupplier zAxis) {
        m_subsystem = subsystem;
        xRotate = xAxis;
        zClimb = zAxis;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }
    @Override
      public void execute() {
        m_subsystem.climb(zClimb.getAsDouble());
        m_subsystem.rotate(xRotate.getAsDouble());
    }
    
    
    public void end(){
        m_subsystem.climb(0);
        m_subsystem.rotate(0);
    }
    
      // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
