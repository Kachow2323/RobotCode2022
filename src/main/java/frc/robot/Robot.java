// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private TalonFX right1;
  private TalonFX right2;
  private TalonFX left1;
  private TalonFX left2;
  private CANSparkMax intake;
  private CANSparkMax shooter;
  private XboxController player1;
  private RobotContainer m_robotContainer;
  private DigitalInput thingyThatGoesBoom;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer(); <-- funnt code line according to Ryan's Repo

    right1 = new TalonFX(4);
    right2 = new TalonFX(2);
    left1 = new TalonFX(3);
    left2 = new TalonFX(1);
    shooter = new CANSparkMax(1, MotorType.kBrushless);
    intake = new CANSparkMax(3, MotorType.kBrushless);
    thingyThatGoesBoom = new DigitalInput(9);
    player1 = new XboxController(0);

    left2.follow(left1); //idk how this part works xd
    right2.follow(right1);
    left1.setInverted(true);
    left2.setInverted(true); 
    right1.setNeutralMode(NeutralMode.Brake);
    right2.setNeutralMode(NeutralMode.Brake);
    left1.setNeutralMode(NeutralMode.Brake);
    left2.setNeutralMode(NeutralMode.Brake);

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
    // This is for Tank Drive - NOT Arcade
    // double leftWheelsControl = player1.getLeftY();
    // double rightWheelsControl = player1.getRightY();
    // right1.set(ControlMode.PercentOutput, rightWheelsControl);
    // left1.set(ControlMode.PercentOutput, leftWheelsControl);
    
    //This is for Arcade Drive 
    double leftSideArcade = player1.getLeftY();
    double rightSideArcade = player1.getRightX();
    double left = leftSideArcade + rightSideArcade ;
    double right = leftSideArcade - rightSideArcade;
    boolean intakeControl = player1.getRightBumper();
    boolean shooterControl = player1.getLeftBumper();
    boolean didItGoBoom = thingyThatGoesBoom.get();

    // Let's do some math, we want wheel control only either left or right 
    if(left > 1.0){
      left = 1.0; //sets a hard limit since it will go over 1
    }else if(right > 1.0){
      right = 1.0; // same, sets a hard limit
    }else if(left < -1.0){
      left = -1.0; // same, sets a hard limit
    }else if(right < -1.0){
      right = -1.0; // same, sets a hard limit
    }

    // Tell either left or right to go!!!
    right1.set(ControlMode.PercentOutput, right);
    left1.set(ControlMode.PercentOutput, left);

    if (intakeControl == false){
      intake.set(0);
    } else if (intakeControl == true){
      intake.set(.75);
    }

    if (shooterControl == false){
      shooter.set(0);
    } else if(shooterControl == true){
      shooter.set(.65);
    }

    //SmartDashboard.putNumber(double, rightWheelsControl);
    SmartDashboard.putNumber("Right Wheel Speed", rightSideArcade);
    SmartDashboard.putNumber("Left Wheel Speed", leftSideArcade);
    SmartDashboard.putBoolean("Digital Input", didItGoBoom);
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
