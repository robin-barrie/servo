package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Chassis extends SubsystemBase {

  private MecanumDrive m_robotDrive;

  private static final int kFrontLeftChannel = 6;//6
  private static final int kFrontRightChannel = 7;
  private static final int kRearLeftChannel = 9;
  private static final int kRearRightChannel = 8;

  private PWMVictorSPX frontLeft;
  private PWMVictorSPX rearLeft;
  private PWMVictorSPX frontRight;
  private PWMVictorSPX rearRight;

    /**
    *
    */
    public Chassis() {

      frontLeft = new PWMVictorSPX(kFrontLeftChannel);
      rearLeft = new PWMVictorSPX(kRearLeftChannel);
      frontRight = new PWMVictorSPX(kFrontRightChannel);
      rearRight = new PWMVictorSPX(kRearRightChannel);

      frontRight.setInverted(true);
      rearRight.setInverted(true);
      frontLeft.setInverted(false);
      rearLeft.setInverted(false);
  
      m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void joystickDrive(XboxController joystick) {
      // Use the joystick X axis for forward movement, Y axis for lateral
      // movement, and Z axis for rotation.
      XboxController m_stick = joystick;
      double LY = -m_stick.getLeftY();
      double LX = -m_stick.getLeftX();
      double RX = m_stick.getRightX();
      if(Math.abs(LY)<0.2){LY=0;}
      if(Math.abs(LX)<0.2){LX=0;}
      if(Math.abs(RX)<0.2){RX=0;}


      m_robotDrive.driveCartesian(LY, LX, RX);
    }


}

