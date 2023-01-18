package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.net.ssl.TrustManager;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shoulder extends SubsystemBase {

private WPI_TalonFX m_shoulderMotor;

public boolean kInverted, kSensorPhase;
public Integer kPid_ID;
public double kP, kI, kD, kIz, kFF, kMaxForward, kMaxReverse, kNomForward, kNomReverse, allowableClosedLoopError;


    /**
    *
    */
    public Shoulder() {
    // initialize motor
      m_shoulderMotor  = new WPI_TalonFX(3, "rio");
      m_shoulderMotor.configFactoryDefault();
   
    // PID coefficients
        kPid_ID = 00;
        kP = .1; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        allowableClosedLoopError = 200;
        kMaxForward = 0.2; 
        kMaxReverse = -0.2;
        kNomForward = 0.0;
        kNomReverse = -0.0;
        kInverted = false;
        kSensorPhase = true;
    
/* Factory Default all hardware to prevent unexpected behaviour */
m_shoulderMotor.configFactoryDefault();
		
/* Config the sensor used for Primary PID and sensor direction */
m_shoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPid_ID,10);

/* Ensure sensor is positive when output is positive */
m_shoulderMotor.setSensorPhase(kSensorPhase);
m_shoulderMotor.setSelectedSensorPosition(0);

/**
 * Set based on what direction you want forward/positive to be.
 * This does not affect sensor phase. 
 */ 
m_shoulderMotor.setInverted(kInverted);
/*
 * Talon FX does not need sensor phase set for its integrated sensor
 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
 * 
 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
 */
    // _talon.setSensorPhase(true);

/* Config the peak and nominal outputs, 12V means full */
m_shoulderMotor.configNominalOutputForward(kNomForward, 10);
m_shoulderMotor.configNominalOutputReverse(kNomReverse, 10);
m_shoulderMotor.configPeakOutputForward(kMaxForward, 10);
m_shoulderMotor.configPeakOutputReverse(kMaxReverse, 10);

/**
 * Config the allowable closed-loop error, Closed-Loop output will be
 * neutral within this range. See Table in Section 17.2.1 for native
 * units per rotation.
 */
m_shoulderMotor.configAllowableClosedloopError(kPid_ID, allowableClosedLoopError, 10);

/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
m_shoulderMotor.config_kF(kPid_ID, kFF, 10);
m_shoulderMotor.config_kP(kPid_ID, kP, 10);
m_shoulderMotor.config_kI(kPid_ID, kI, 10);
m_shoulderMotor.config_kD(kPid_ID, kD, 10);


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        SmartDashboard.putNumber("actual pos", m_shoulderMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("actual velocity", m_shoulderMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("actual percent output", m_shoulderMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("closed loop error", m_shoulderMotor.getClosedLoopError());
        SmartDashboard.putNumber("closed loop target", m_shoulderMotor.getClosedLoopTarget());

        SmartDashboard.putString("control mode", m_shoulderMotor.getControlMode().toString());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    
      public void setPosition(double pos){
        m_shoulderMotor.set(TalonFXControlMode.Position, pos);
        SmartDashboard.putNumber("target pos", pos);
        return;
      }
      public void setSpeed(double speed){
        m_shoulderMotor.set(TalonFXControlMode.PercentOutput, speed);
        return;
      }
      public double getOutput(){
      		/* Get Talon's current output percentage */
		      double motorOutput = m_shoulderMotor.getMotorOutputPercent();
        return motorOutput;
        }
}