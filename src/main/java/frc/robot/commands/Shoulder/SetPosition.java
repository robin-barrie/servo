package frc.robot.commands.Shoulder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shoulder;

import java.util.function.DoubleSupplier;

/**
 *
 */
public class SetPosition extends InstantCommand {
    private final Shoulder m_shoulder;
    private final Double m_position;

    public SetPosition(Double pos, Shoulder shoulder) {
        m_shoulder = shoulder;
        m_position = pos;
        
        addRequirements(m_shoulder);    
    }

    // Called once when this command runs
    /*  
     */
    @Override
    public void initialize() {
        /*1 Rotation = 2048 u/rev in either direction */
        m_shoulder.setPosition(m_position);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
