// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.util.sendable.SendableRegistry;
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


 // int leftMotor = 3; // this is the motor CAN ID 
  int diff_drive_right_motor_canID = 2; //this is the motor CAN ID 
  int diff_drive_left_motor_canID = 3;

  public SparkMax chassis_motor_left = new SparkMax(diff_drive_left_motor_canID, SparkLowLevel.MotorType.kBrushed); // 0 is the RIO PWM port this is connected to
  public SparkMax chassis_motor_right = new SparkMax(diff_drive_right_motor_canID, SparkLowLevel.MotorType.kBrushed);


  private Joystick chassis_stick_left;
  private Joystick chassis_stick_right;



  private final RobotContainer m_robotContainer;
    
      /**
       * This function is run when the robot is first started up and should be used for any
       * initialization code.
       */
      public Robot() {
        // this is to test the motor controllers 
//SendableRegistry.addChild(m_robotDrive, LMspark);
       // SendableRegistry.addChild(m_robotDrive,  RMspark);
     // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
       // RMspark.setInverted(true);
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
m_myRobot = new DifferentialDrive(chassis_motor_left, chassis_motor_right);
//       chassis_motor_left.configure(SparkBaseConfig, SparkBase.ResetMode, SparkBase.PersistMode);

chassis_stick_left = new Joystick(0);
//chassis_stick_right = new Joystick(1);


        m_robotContainer = new RobotContainer();
      }
   
    /* private Spark SparkMax(int canID, MotorType motorType) {
        // TODO Auto-generated method stub
        Spark motor = new SparkMax (canID, motorType);
        
        SparkBaseConfig config = new SparkBaseConfig();
        config.inverted(false);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        return motor;
      } */
  
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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
   m_myRobot.arcadeDrive(chassis_stick_left.getY(), chassis_stick_left.getX());
   


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
