package frc.robot.commands.Shoulder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shoulder;

import java.util.function.DoubleSupplier;

/**
 *
 */
public class SetSpeed extends InstantCommand {
    private final Shoulder m_shoulder;
    private final Double m_speed;

    public SetSpeed(Double speed, Shoulder shoulder) {
        m_shoulder = shoulder;
        m_speed = speed;
        addRequirements(m_shoulder);    
    }

    // Called once when this command runs
    @Override
    public void initialize() {
        m_shoulder.setSpeed(this.m_speed);
        SmartDashboard.putNumber("target speed", m_speed);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
