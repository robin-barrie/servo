package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Elbo extends SubsystemBase {

private Servo servo2;
private PWMVictorSPX motorController3;
private PWMVictorSPX motorController5;

    /**
    *
    */
    public Elbo() {
 servo2 = new Servo(2);
 addChild("Servo 2", servo2);
 
 motorController5 = new PWMVictorSPX(5);
 addChild("Motor Controller 5",motorController5);
motorController3 = new PWMVictorSPX(3);
 addChild("Motor Controller 3",motorController3);
 motorController3.setInverted(false);
 motorController5.setInverted(false);
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

    public void setMotor(double speed){
      motorController3.set(speed);
      motorController5.set(speed);
      SmartDashboard.putNumber("sppeed", speed);
      //leftServo.set(pos);
    }

    public void setServo(double pos){
        servo2.set(pos);
        //leftServo.set(pos);
      }
    
      public void setServoAngle(double angle){
        servo2.setAngle(angle);
        //leftServo.setAngle(angle);
      }
    
      public void setServoPWM(int pwm){
        servo2.setRaw(pwm);
      }
    
      public double getPosition(){
        //setBounds must be set first??  same as setPosition...
        return servo2.get();
      }
    
      public double getAngle(){
        return servo2.getAngle();
      }
    
      public void setServoBounds(){
        //servo2.setBounds(max, deadbandMax, center, deadbandMin, min);
        //servo2.setBounds(max, deadbandMax, center, deadbandMin, min);
      }

}

