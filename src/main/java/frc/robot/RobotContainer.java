package frc.robot;

import frc.robot.commands.*;
import frc.robot.commands.Chassis.*;
import frc.robot.commands.Elbo.*;
import frc.robot.commands.Shoulder.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

// The robot's subsystems
    public final Elbo m_elbo = new Elbo();
    public final Shoulder m_shoulder = new Shoulder();
    public final Chassis m_chassis = new Chassis();
    public final Simm m_simulate = new Simm();

// Joysticks
private final XboxController xboxController1 = new XboxController(0);


  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {


    //m_elbo.setDefaultCommand(new JoystickDriveMotor(xboxController1, m_elbo));
    m_chassis.setDefaultCommand(new JoystickDriveMecanum(xboxController1, m_chassis));

    

    // Smartdashboard Subsystems


    // SmartDashboard Buttons
    SmartDashboard.putData("Command 1", new Command1());
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    SmartDashboard.putData("Wait Command 1", new WaitCommand1(1));
    SmartDashboard.putData("Set Angle", new SetPosition(10.0, m_shoulder));

    // Configure the button bindings
    configureButtonBindings();
    // Create some buttons
    final JoystickButton A = new JoystickButton(xboxController1, 1);
    final JoystickButton B = new JoystickButton(xboxController1, 2);
    final JoystickButton X = new JoystickButton(xboxController1, 3);
    final JoystickButton Y = new JoystickButton(xboxController1, 4);
    final JoystickButton dpadUp = new JoystickButton(xboxController1, 5);
    final JoystickButton dpadRight = new JoystickButton(xboxController1, 6);
    final JoystickButton dpadDown = new JoystickButton(xboxController1, 7);
    final JoystickButton dpadLeft = new JoystickButton(xboxController1, 8);
    final JoystickButton l2 = new JoystickButton(xboxController1, 9);
    final JoystickButton r2 = new JoystickButton(xboxController1, 10);
    final JoystickButton l1 = new JoystickButton(xboxController1, 11);
    final JoystickButton r1 = new JoystickButton(xboxController1, 12);

 // Connect the buttons to commands
   A.onTrue(new SetPosition(0.0, m_shoulder));
   B.onTrue(new SetPosition(2048.0, m_shoulder));
   X.onTrue(new SetSpeed(0.5, m_shoulder));
   Y.onTrue(new SetSpeed(0.0, m_shoulder));
   dpadUp.onTrue(new ButtonDriveMotor(0.5, m_elbo));
   dpadDown.onTrue(new ButtonDriveMotor(-0.5, m_elbo));

    // Configure default commands


    // Configure autonomous sendable chooser
    
    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
// Create some buttons
final JoystickButton xboxButtonA = new JoystickButton(xboxController1, XboxController.Button.kA.value);        
xboxButtonA.onTrue(new Command1().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                        
  }

 public XboxController getXboxController1() {
      return xboxController1;
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
  

}