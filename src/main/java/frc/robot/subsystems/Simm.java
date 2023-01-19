package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 *
 */
public class Simm extends SubsystemBase {

  private WPI_TalonFX m_shoulderMotor;
  private WPI_TalonFX m_elboMotor;

public boolean kInverted, kSensorPhase;
public Integer kPid_ID;
public double kP, kI, kD, kIz, kFF, kMaxForward, kMaxReverse, kNomForward, kNomReverse, allowableClosedLoopError;
private final double kArmEncoderDistPerPulse;
  // Simulation classes help us simulate what's going on, including gravity.
private static final double m_armReduction = 600;
private static final double m_arm_topMass = 10.0; // Kilograms
private static final double m_arm_topLength = Units.inchesToMeters(38.5);
private static final double m_arm_bottomMass = 4.0; // Kilograms
private static final double m_arm_bottomLength = Units.inchesToMeters(27);

private static final int m_arm_top_min_angle = -75; 
private static final int m_arm_top_max_angle = 260; 
private static final int m_arm_bottom_min_angle = 30; 
private static final int m_arm_bottom_max_angle = 150; 

  //SETPOINTS FOR PRESETS MODE (Uses Virtual 4 Bar Mode for smooth movement)
private static final int stowedBottom = 90;
private static final int stowedTop = 260;

private static final int intakeBottom = 135;
private static final int intakeTop = 265;

private static final int doubleSubstationBottom = 60;
private static final int doubleSubstationTop = 185;

private static final int scoreFloorBottom = 120;
private static final int scoreFloorTop = 255;

private static final int scoreMidBottom = 95;
private static final int scoreMidTop = 195;

private static final int scoreHighBottom = 135;
private static final int scoreHighTop = 160;


private final DCMotor m_armGearbox;
//Can't create encoder for sim from falcon internal encoder. Needed to create external encoder and pidcontrol to use with sim.
private final Encoder m_topEncoder, m_bottomEncoder;
private final ProfiledPIDController m_topController, m_bottomController;
public double pidOutputTop, pidOutputBottom;

  // The P gain for the PID controller that drives this arm. 
  private static final double kArmKp = 40.0;
  private static final double kArmKi = 0.0;

  private final SingleJointedArmSim m_arm_topSim, m_arm_bottomSim;
  private EncoderSim m_topEncoderSim, m_bottomEncoderSim;

  private final Mechanism2d m_mech2d;
  private final MechanismRoot2d midNodeHome;
  private final MechanismLigament2d MidNode;
  private final MechanismRoot2d highNodeHome;
  private final MechanismLigament2d HighNode;
  private final MechanismRoot2d gridHome;
  private final MechanismLigament2d GridNode;
  private final MechanismRoot2d dsHome;
  private final MechanismLigament2d DSRampor;
  private final MechanismRoot2d m_armPivot;
  private final MechanismLigament2d m_arm_bottom;
  private final MechanismLigament2d m_arm_tower;
 
  private final MechanismLigament2d m_aframe_1;
  private final MechanismLigament2d m_bumper;
  private final MechanismLigament2d m_arm_top;
  private final MechanismLigament2d m_intake;

SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

    /**
    *
    */
    public Simm() {

        // initialize motor
        m_shoulderMotor  = new WPI_TalonFX(3, "rio");
        m_elboMotor  = new WPI_TalonFX(5, "rio");

     
      // PID coefficients
      //SET UP SEPERATE FOR EACH JOINT
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
      
  /* Factory Default all hardware to prevent unexpected behaviour */
  m_shoulderMotor.configFactoryDefault();
  m_elboMotor.configFactoryDefault();
      
  /* Config the sensor used for Primary PID and sensor direction */
  m_shoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPid_ID,10);
  m_elboMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPid_ID,10);
  
  /* Set initial sensor position */
  m_shoulderMotor.setSelectedSensorPosition(0);
  m_elboMotor.setSelectedSensorPosition(0);
  
  /**
   * Set based on what direction you want forward/positive to be.
   * This does not affect sensor phase. 
   */ 
  m_shoulderMotor.setInverted(kInverted);
  m_elboMotor.setInverted(kInverted);

  /*
   * Talon FX does not need sensor phase set for its integrated sensor
   * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
   * and the user calls getSelectedSensor* to get the sensor's position/velocity.
   */
      // _talon.setSensorPhase(true);
  
  /* Config the peak and nominal outputs, 12V means full */
  m_shoulderMotor.configNominalOutputForward(kNomForward, 10);
  m_shoulderMotor.configNominalOutputReverse(kNomReverse, 10);
  m_shoulderMotor.configPeakOutputForward(kMaxForward, 10);
  m_shoulderMotor.configPeakOutputReverse(kMaxReverse, 10);
  m_elboMotor.configNominalOutputForward(kNomForward, 10);
  m_elboMotor.configNominalOutputReverse(kNomReverse, 10);
  m_elboMotor.configPeakOutputForward(kMaxForward, 10);
  m_elboMotor.configPeakOutputReverse(kMaxReverse, 10);
  
  /**
   * Config the allowable closed-loop error, Closed-Loop output will be
   * neutral within this range. See Table in Section 17.2.1 for native
   * units per rotation.
   */
  m_shoulderMotor.configAllowableClosedloopError(kPid_ID, allowableClosedLoopError, 10);
  m_elboMotor.configAllowableClosedloopError(kPid_ID, allowableClosedLoopError, 10);
  
  /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
  m_shoulderMotor.config_kF(kPid_ID, kFF, 10);
  m_shoulderMotor.config_kP(kPid_ID, kP, 10);
  m_shoulderMotor.config_kI(kPid_ID, kI, 10);
  m_shoulderMotor.config_kD(kPid_ID, kD, 10);
  m_elboMotor.config_kF(kPid_ID, kFF, 10);
  m_elboMotor.config_kP(kPid_ID, kP, 10);
  m_elboMotor.config_kI(kPid_ID, kI, 10);
  m_elboMotor.config_kD(kPid_ID, kD, 10);
  
  
/***************************************************************************************************************************************
ADDED BOTH MOTORS HERE AS AN ARM SUBSYSTEM????

  public void robotInit() {
    m_topEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    m_bottomEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);

      }


************************************************************************************************************************************************/
m_topController = new ProfiledPIDController(kArmKp, kArmKi, 0, new TrapezoidProfile.Constraints(2, 5));
m_bottomController = new ProfiledPIDController(kArmKp, kArmKi, 0, new TrapezoidProfile.Constraints(2, 5));
m_topEncoder = new Encoder(0, 1);
m_bottomEncoder = new Encoder(2, 3);


  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // The arm gearbox represents a gearbox containing two Falcon500 motors.
  m_armGearbox = DCMotor.getFalcon500(2);





  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
 m_arm_topSim =
      new SingleJointedArmSim(
          m_armGearbox,
          m_armReduction,
          SingleJointedArmSim.estimateMOI(m_arm_topLength, m_arm_topMass),
          m_arm_topLength,
          Units.degreesToRadians(m_arm_top_min_angle),
          Units.degreesToRadians(m_arm_top_max_angle),
          m_arm_topMass,
          false,
          VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );
 m_arm_bottomSim =
          new SingleJointedArmSim(
              m_armGearbox,
              m_armReduction,
              SingleJointedArmSim.estimateMOI(m_arm_bottomLength, m_arm_bottomMass),
              m_arm_bottomLength,
              Units.degreesToRadians(m_arm_bottom_min_angle),
              Units.degreesToRadians(m_arm_bottom_max_angle),
              m_arm_bottomMass,
              true,
              VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
              );
  m_topEncoderSim = new EncoderSim(m_topEncoder);
  m_bottomEncoderSim = new EncoderSim(m_bottomEncoder);

 // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  
 m_mech2d = new Mechanism2d(90, 90);
 midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0);
 MidNode = midNodeHome.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
 highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0);
 HighNode = highNodeHome.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
 gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0);
 GridNode = gridHome.append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
 dsHome = m_mech2d.getRoot("Double Substation Home", 49.75, 37);
 DSRampor = dsHome.append(new MechanismLigament2d("Double Substation Ramp", 13.75, 180, 10, new Color8Bit(Color.kWhite)));
 m_armPivot = m_mech2d.getRoot("ArmPivot", 65, 21.75);
 m_arm_bottom =
     m_armPivot.append(
           new MechanismLigament2d(
             "Arm Bottom",
             27, 
             -90, 
             10, 
             new Color8Bit(Color.kGold)));
m_arm_tower = m_armPivot.append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kSilver)));

m_aframe_1 = m_armPivot.append(new MechanismLigament2d("aframe1", 24, -50, 10, new Color8Bit(Color.kSilver)));
m_bumper = gridHome.append(new MechanismLigament2d("Bumper", 30.5, 0, 60, new Color8Bit(Color.kRed)));
m_arm_top =
     m_arm_bottom.append(
         new MechanismLigament2d(
             "Arm Top",
             28.5 + 3.0,
             Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
             10,
             new Color8Bit(Color.kPurple)));
m_intake =
   m_arm_top.append(
       new MechanismLigament2d(
           "Intake",
           7,
           Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
           40,
           new Color8Bit(Color.kWhite)));




    SmartDashboard.putNumber("Setpoint top (degrees)", 90);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", 90);

    controlMode.setDefaultOption("Presets (Setpoints)", 0);
    controlMode.addOption("Virtual Four Bar", 1);
    controlMode.addOption("Manual Angle Adjust", 2);
      SmartDashboard.putData(controlMode);

    presetChooser.setDefaultOption("Starting Position", 0);
    presetChooser.addOption("Floor Intake Position", 1);
    presetChooser.addOption("Double Substation Intake", 2);
    presetChooser.addOption("Floor Node Score", 3);
    presetChooser.addOption("Mid Node Score", 4);
    presetChooser.addOption("High Node Score", 5);
      SmartDashboard.putData(presetChooser);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        

        switch(controlMode.getSelected()){
          case 1:
            // Here, we run PID control where the top arm acts like a four-bar relative to the bottom. 
              break;
          case 2:
            // Here, we run a PID control basesd on setpoints entered into SmartDashboard
            pidOutputTop = m_topController.calculate(m_topEncoder.getDistance(), Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint top (degrees)", 0), m_arm_top_min_angle, m_arm_top_max_angle)));
            m_elboMotor.setVoltage(pidOutputTop);
    
            pidOutputBottom = m_bottomController.calculate(m_bottomEncoder.getDistance(), Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0), m_arm_bottom_min_angle, m_arm_bottom_max_angle)));
            m_shoulderMotor.setVoltage(pidOutputBottom);
            break;
          default: //also case 0
            //Run PID based on drop down menu of setpoints
            int topSetpoint, bottomSetpoint;
            switch(presetChooser.getSelected()){
              case 0:
                topSetpoint = stowedTop;
                bottomSetpoint = stowedBottom;
                break;
              case 1:
                topSetpoint = intakeTop;
                bottomSetpoint = intakeBottom;
                break;
              case 2:
                topSetpoint = doubleSubstationTop;
                bottomSetpoint = doubleSubstationBottom;
                break;
              case 3:
                topSetpoint = scoreFloorTop;
                bottomSetpoint = scoreFloorBottom;
                break;
              case 4:
                topSetpoint = scoreMidTop;
                bottomSetpoint = scoreMidBottom;
                break;
              case 5:
                topSetpoint = scoreHighTop;
                bottomSetpoint = scoreHighBottom;
                break;
              default:
                topSetpoint = stowedTop;
                bottomSetpoint = stowedBottom;
                break;
            }
            // Here, we run PID control where the arm moves to the selected setpoint.
            pidOutputTop = m_topController.calculate(m_topEncoder.getDistance(), Units.degreesToRadians(topSetpoint - bottomSetpoint));
            m_elboMotor.setVoltage(pidOutputTop);
            SmartDashboard.putNumber("Setpoint bottom (degrees)", bottomSetpoint);
            SmartDashboard.putNumber("Setpoint top (degrees)", topSetpoint);
            pidOutputBottom = m_bottomController.calculate(m_bottomEncoder.getDistance(), Units.degreesToRadians(bottomSetpoint));
            m_shoulderMotor.setVoltage(pidOutputBottom);
            break;
        }
      }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

            // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_arm_topSim.setInput(m_elboMotor.get() * RobotController.getBatteryVoltage());
    m_arm_bottomSim.setInput(m_shoulderMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_arm_topSim.update(0.020);
    m_arm_bottomSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_topEncoderSim.setDistance(m_arm_topSim.getAngleRads());
    m_bottomEncoderSim.setDistance(m_arm_bottomSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_arm_topSim.getCurrentDrawAmps() + m_arm_bottomSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm_top.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));
    m_arm_bottom.setAngle(Units.radiansToDegrees(m_arm_bottomSim.getAngleRads()));


    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
  }