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
public class JoystickDriveMotor extends CommandBase {
    Elbo m_elbo;
    XboxController m_joystick;

    public JoystickDriveMotor(XboxController xboxController1, Elbo elbo) {
        m_elbo = elbo;
        m_joystick = xboxController1;
        addRequirements(m_elbo);    
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = - m_joystick.getLeftY();
        if (Math.abs(speed) < 0.2){speed = 0;}
        m_elbo.setMotor(speed);
        SmartDashboard.putNumber("joysitcj", speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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