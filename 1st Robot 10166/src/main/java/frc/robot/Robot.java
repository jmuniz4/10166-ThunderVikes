// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */

public class Robot extends TimedRobot {
private Command m_autonomousCommand;


private DifferentialDrive m_myRobot;

private final Timer timer = new Timer();

 // int leftMotor = 3; // this is the motor CAN ID 
  int diff_drive_motor_canID_right = 2; //this is the motor CAN ID 
  int diff_drive_motor_canID_right_follow = 3; //this is the motor CAN ID 
  
  int diff_drive_motor_canID_left = 8;
  int diff_drive_motor_canID_left_follow = 6;

  int elevator_motor_canID_left = 4;
  int elevator_motor_canID_right = 9;

  int claw_motor_canID_left = 7;
  int claw_motor_canID_right = 10;


  public SparkMax chassis_motor_left = new SparkMax(diff_drive_motor_canID_left, SparkLowLevel.MotorType.kBrushed); // 0 is the RIO PWM port this is connected to
  public SparkMax chassis_motor_left_follow = new SparkMax(diff_drive_motor_canID_left_follow, SparkLowLevel.MotorType.kBrushed);
  
  public SparkMax chassis_motor_right = new SparkMax(diff_drive_motor_canID_right, SparkLowLevel.MotorType.kBrushed);
  public SparkMax chassis_motor_right_follow = new SparkMax(diff_drive_motor_canID_right_follow, SparkLowLevel.MotorType.kBrushed);
  
  public SparkFlex claw_motor_left = new SparkFlex(claw_motor_canID_left, SparkLowLevel.MotorType.kBrushed);
  public SparkFlex claw_motor_right = new SparkFlex(claw_motor_canID_right, SparkLowLevel.MotorType.kBrushed);

  
  public SparkMax elevat_motor_left = new SparkMax(elevator_motor_canID_left, SparkLowLevel.MotorType.kBrushless); 
  public SparkMax elevat_motor_right = new SparkMax(elevator_motor_canID_right, SparkLowLevel.MotorType.kBrushless);  
  public RelativeEncoder elevatorEncoder = elevat_motor_left.getEncoder();
  public RelativeEncoder elevatorREncoder = elevat_motor_right.getEncoder();


 
  
  private XboxController controller1 = new XboxController(0);


  private final RobotContainer m_robotContainer;
    
      /**
       * This function is run when the robot is first started up and should be used for any
       * initialization code.
       */
      @SuppressWarnings("deprecation")
      public Robot() {
        
     // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
  
        
        m_myRobot = new DifferentialDrive(chassis_motor_left, chassis_motor_right); 
        m_myRobot = new DifferentialDrive(chassis_motor_left_follow, chassis_motor_right_follow); // we add this one


        //       chassis_motor_left.configure(SparkBaseConfig, SparkBase.ResetMode, SparkBase.PersistMode);
       
       // chassis_motor_left_follow.set(chassis_motor_left.get());
       // chassis_motor_right_follow.set(chassis_motor_right.get());
        

       // controller1 = new XboxController(0);
      

        //chassis_stick_right = new Joystick(1);
       
    

        
  
        chassis_motor_right.setInverted(true);// we change chassis_motor_right to left to make the chassis move forward
        chassis_motor_right_follow.setInverted(true);


        m_robotContainer = new RobotContainer();
      }

      public void runMotorAuto(double xSpeed,double RotaionSpeed){
        m_myRobot.arcadeDrive(xSpeed, RotaionSpeed);

      }

      public void runTeleop(XboxController controller){

        double Xinput = controller.getLeftY();
        double Rotate = controller.getLeftX();

        m_myRobot.arcadeDrive(Xinput, Rotate);
      }

      public double getEncoderDrive(){
       double encoderPosition =  chassis_motor_left.getEncoder().getPosition();

       return encoderPosition;

      }
   
    
  
    /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //double time = Timer.getFPGATimestamp();//add

    if(timer.advanceIfElapsed(0.1)){
      runMotorAuto(.25, 0);
    }else if (timer.advanceIfElapsed(3)){
      runMotorAuto(0, 0);
    }

  
  
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.



    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    runTeleop(controller1);



    double initialencoder;
    int Level1;
    int Level2;
    int Level3;
    double midcoder;
    double topcoder;
    double initialenencoder;

  //m_myRobot.arcadeDrive(controller1.getLeftY(), controller1.getLeftX());
 
   SmartDashboard.putNumber("encoder position",elevatorEncoder.getPosition());
   SmartDashboard.putNumber("encoder velocity",elevatorEncoder.getVelocity());


   //when we press left stick button robot turn left
   /**  if(controller.getLeftStickButton()){
     chassis_motor_left.set(-0.5);
     }else if(controller.getLeftStickButton()){
     chassis_motor_left.set(0);
    }

    if(controller.getRightStickButton()){
      chassis_motor_right.set(0.5);
      }else if(controller.getRightStickButton()){
      chassis_motor_right.set(0);
      }*/

 
   /* if(controller.getAButtonPressed()){
    initialencoder = elevatorEncoder.getPosition();
    while((elevatorEncoder.getPosition() - initialencoder) <= 0.1){
    elevat_motor_left.set(-0.5);
    elevat_motor_right.set(-0.5);
    }
   }else if(controller.getAButtonReleased()){
    midcoder = elevatorEncoder.getPosition();
    while ((elevatorEncoder.getPosition() - midcoder ) <= 0.1) {
    elevat_motor_left.set(0.5);
    elevat_motor_right.set(0.5);
    }
  
   } */

   /*if(controller.getAButtonPressed()){
    initialencoder = elevatorEncoder.getPosition();
    elevat_motor_left.set(0.5);
    //elevat_motor_right.set(0.5);
   }
   else{
    elevat_motor_left.set(0);
   }
   */
  claw_motor_left.setInverted(true);
   
   if(controller1.getBButtonPressed()){
    elevat_motor_left.set(5);
    elevat_motor_right.set(5);
  }else if(controller1.getBButtonReleased()){
    elevat_motor_left.set(0);
    elevat_motor_right.set(0);
   }
  
   if(controller1.getAButtonPressed()){
    elevat_motor_left.set(-5);
    elevat_motor_right.set(-5);
  }else if(controller1.getAButtonReleased()){
    elevat_motor_left.set(0);
    elevat_motor_right.set(0);
   
  }

   if(controller1.getLeftBumperButtonPressed()){
    claw_motor_left.set(5);
    claw_motor_right.set(5);
  }else if(controller1.getLeftBumperButtonReleased()){
    claw_motor_left.set(0);
    claw_motor_right.set(0);
   }
  
   if(controller1.getRightBumperButtonPressed()){
    claw_motor_left.set(-5);
    claw_motor_right.set(-5);
  }else if(controller1.getRightBumperButtonReleased()){
    claw_motor_left.set(0);
    claw_motor_right.set(0);
   
  }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
