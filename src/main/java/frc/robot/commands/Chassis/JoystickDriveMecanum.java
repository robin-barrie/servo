package frc.robot.commands.Chassis;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elbo;

import java.util.function.DoubleSupplier;

/**
 *
 */
public class JoystickDriveMecanum extends CommandBase {
    Chassis m_chassis;
    XboxController m_joystick;

    public JoystickDriveMecanum(XboxController xboxController1, Chassis chassis) {
        m_chassis = chassis;
        m_joystick = xboxController1;
        addRequirements(m_chassis);    
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_chassis.joystickDrive(m_joystick);
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