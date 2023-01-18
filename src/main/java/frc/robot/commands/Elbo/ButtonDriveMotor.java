package frc.robot.commands.Elbo;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Elbo;

import java.util.function.DoubleSupplier;

/**
 *
 */
public class ButtonDriveMotor extends CommandBase {
    Elbo m_elbo;
    Double m_speed;

    public ButtonDriveMotor(Double speed, Elbo elbo) {
        m_elbo = elbo;
        m_speed = speed;
        addRequirements(m_elbo);    
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elbo.setMotor(m_speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_elbo.setMotor(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}