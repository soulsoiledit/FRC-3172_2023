// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.net.http.HttpClient.Redirect;

import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDriveFront = new PWMSparkMax(0);
  private final PWMSparkMax m_leftDriveRear = new PWMSparkMax(1);
  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftDriveFront, m_leftDriveRear);

  private final PWMSparkMax m_rightDriveFront = new PWMSparkMax(2);
  private final PWMSparkMax m_rightDriveRear = new PWMSparkMax(3);
  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightDriveFront, m_rightDriveRear);

  private final PWMSparkMax m_moveArm1 = new PWMSparkMax(5);
  private final PWMSparkMax m_moveArm2 = new PWMSparkMax(6);

  private final PWMSparkMax m_elevatorMotor1 = new PWMSparkMax(7);
  private final PWMSparkMax m_elevatorMotor2 = new PWMSparkMax(8);

  Relay clawActuator =  new Relay(1);
  Servo clawServo = new Servo(4);

  Servo dropServo = new Servo(9);

  AHRS gyro = new AHRS(Port.kUSB1);

  public static boolean retractArm;

  public static int clawAngle;

  public static int dropAngle;

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  //private final DigitalOutput dio0Out = new DigitalOutput(0);
  //private final DigitalOutput dio1Out = new DigitalOutput(1);
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
    m_leftDriveFront.setInverted(true);
    Shuffleboard.getTab("Gyro").add(gyro);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double error,p,i,d,motor,error_last,setpoint,sensor;
    sensor = gyro.getPitch();
    
    setpoint = 0;
    error_last = 0;
    i = 0;
    double dt = 50;
    double kP = 1;
    double kI = 0.4;
    double kD = 4;
    boolean motoron = false;

    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else{
      error = setpoint - sensor;
      p = error * kP;
      i += (error * kI);
      d = ((error - error_last) / dt) * kD;
      motor = p + i + d;
      if(motor > 100){
        motor = 100;
      }
      double motorout = -motor / 100;
      if(motorout < 0.1 && motorout > -0.1){
        m_robotDrive.arcadeDrive(0, 0.0, false);
        motoron = false;
      }
      else{
        m_robotDrive.arcadeDrive(motorout, 0.0, false);
        motoron = true;
      }
      
      SmartDashboard.putNumber("Auto PID Motor Output", motorout);
      SmartDashboard.putBoolean("Auto Motor Running", motoron);

      error_last = error; 
      
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {
        //TODO Auto-generated catch block
        e.printStackTrace();
      }
    } 
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    //Sets the servo angle to 0 degrees everytime the robot enters teleoperated mode
    gyro.resetDisplacement();
    gyro.calibrate();
    clawAngle = 90;
    dropAngle = 90;
    retractArm = false;
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());

    double error,p,i,d,motor,error_last,setpoint,sensor;
    sensor = gyro.getPitch();
    
    setpoint = 0;
    error_last = 0;
    i = 0;
    double dt = 50;
    double kP = 1;
    double kI = 0.4;
    double kD = 4;
    boolean motoron = false;

    SmartDashboard.putNumber("Is dx", gyro.getPitch());
    SmartDashboard.putNumber("Is dy", gyro.getDisplacementY() * 254);
    SmartDashboard.putNumber("Is dz", gyro.getDisplacementZ() * 254);

    if(m_controller.getAButton()){
      error = setpoint - sensor;
      p = error * kP;
      i += (error * kI);
      d = ((error - error_last) / dt) * kD;
      motor = p + i + d;
      if(motor > 100){
        motor = 100;
      }
      double motorout = -motor / 100;
      if(motorout < 0.1 && motorout > -0.1){
        m_robotDrive.arcadeDrive(0, 0.0, false);
        motoron = false;
      }
      else{
        m_robotDrive.arcadeDrive(motorout, 0.0, false);
        motoron = true;
      }
      
      SmartDashboard.putNumber("Tele PID Motor Output", motorout);
      SmartDashboard.putBoolean("Tele Motor Running", motoron);

      error_last = error; 
      
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }

      //Gets the button pressesd on the D-Pad as a number (4~gt4 degree increments) as the dPad variable
      int dPad = m_controller.getPOV(0);

      double triggerValueRight = m_controller.getRightTriggerAxis();
      double triggerValueLeft = m_controller.getLeftTriggerAxis();

      boolean aButton = m_controller.getAButton();
      boolean yButton = m_controller.getYButton();


      if(aButton && yButton){
        m_moveArm1.set(0);
        m_moveArm2.set(0);
      }
      if(aButton){
        m_moveArm1.set(0.5);
        m_moveArm2.set(0.5);
      }
      if(yButton){
        m_moveArm1.set(-0.5);
        m_moveArm2.set(-0.5);
      }

      if(triggerValueLeft > 0 && triggerValueRight > 0){
        m_elevatorMotor1.set(0);
        m_elevatorMotor2.set(0);
      }
      if(triggerValueLeft == 0 && triggerValueRight == 0){
        m_elevatorMotor1.set(0);
        m_elevatorMotor2.set(0);
      }
      if(triggerValueRight > 0){
        m_elevatorMotor1.set((triggerValueRight / 1.5));
        m_elevatorMotor2.set((-triggerValueRight / 1.5));
      }
      if(triggerValueLeft > 0){
        m_elevatorMotor1.set((-triggerValueLeft / 1.5));
        m_elevatorMotor2.set((triggerValueLeft / 1.5));
      }

    // Testing DIO
    // boolean bButton = m_controller.getBButton();

    //Moves the servo controlling the claw based on the dPad variable
    if(dPad == 0){
      // dio0Out.set(true);
      // dio1Out.set(false);
      System.out.println("Extending");
      clawActuator.set(Relay.Value.kReverse); 
      retractArm = true;
    }
    else if(dPad == 180){
      //   dio1Out.set(true);
      //   dio0Out.set(false);
      System.out.println("Retracting");
      clawActuator.set(Relay.Value.kForward);
      retractArm = false;
    }
    else {
      clawActuator.set(Relay.Value.kOff);
      // dio0Out.set(true);
      // dio1Out.set(true);
      retractArm = false;
    }
    if (dPad == 90) {
      clawAngle += 4;
    }
    if (dPad == 270) {
      clawAngle -= 4;
    }

    if (m_controller.getXButton()) {
      clawAngle = 150;
    }

    if (m_controller.getYButton()) {
      clawAngle = 90;
    }

    if(clawAngle > 150){
      clawAngle = 150;
    }
    if(clawAngle < 90){
      clawAngle = 90;
    }

    if(m_controller.getBButtonPressed()){
      if(dropAngle == 90){
        dropAngle = 0;
      }
      else if(dropAngle == 0){
        dropAngle = 90;
      }
    }

    clawServo.setAngle(clawAngle);
    dropServo.setAngle(dropAngle);
    SmartDashboard.putBoolean("Retracting Arm", retractArm);
    SmartDashboard.putNumber("D-Pad Value", dPad);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
