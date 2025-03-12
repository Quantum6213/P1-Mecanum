// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** This is a program that shows how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  private static final int kLeftLeaderPort = 3;
  private static final int kLeftFollowerPort = 2;
  private static final int kRightLeaderPort = 4;
  private static final int kRightFollowerPort = 1;
  private static final int ArmMotorPort = 5;
  private static final int WristMotorPort = 6;
  private static final int ClawMotorPort = 7;

  SparkMax leftLeader;
  SparkMax leftFollower;
  SparkMax rightLeader;
  SparkMax rightFollower;
  SparkMax ArmMotor;
  SparkMax wristMotor;
  SparkMax ClawMotor;
  Joystick joystick;
  // set the time
  private final Timer mTimer=new Timer();

  private static final int kJoystickChannel = 0;

  private final MecanumDrive m_robotDrive;
  private final Joystick m_stick;
  
  //set encoder variables
  private boolean ButtonPushed = false;
  private boolean StayMode = false;
  private double SaveValue = 0;

  /** Called once at the beginning of the robot program. */
  public Robot() {
   
    // initialize the motors
    leftLeader = new SparkMax (kLeftLeaderPort,  SparkLowLevel.MotorType.kBrushless);
    leftFollower = new SparkMax (kLeftFollowerPort, SparkLowLevel.MotorType.kBrushless);
    rightLeader = new SparkMax (kRightLeaderPort, SparkLowLevel.MotorType.kBrushless);
    rightFollower = new SparkMax (kRightFollowerPort, SparkLowLevel.MotorType.kBrushless);
    wristMotor = new SparkMax(WristMotorPort, SparkLowLevel.MotorType.kBrushless);
    ArmMotor = new SparkMax(ArmMotorPort, SparkLowLevel.MotorType.kBrushless);
    ClawMotor = new SparkMax(ClawMotorPort, SparkLowLevel.MotorType.kBrushless);



    // clear faults to begin with
    leftLeader.clearFaults();
    leftFollower.clearFaults();
    rightLeader.clearFaults();
    rightFollower.clearFaults();
    wristMotor.clearFaults();
    ArmMotor.clearFaults();
    ClawMotor.clearFaults();

    // set up configuration
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    leftLeaderConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(true);
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(true);
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightLeaderConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(false);
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(false);
    SparkMaxConfig ArmMotorConfig = new SparkMaxConfig();
    ArmMotorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(false);

    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     
    m_robotDrive = new MecanumDrive(leftLeader::set, leftFollower::set, rightLeader::set, rightFollower::set);

    m_stick = new Joystick(kJoystickChannel); 

    SendableRegistry.addChild(m_robotDrive, leftLeader);
    SendableRegistry.addChild(m_robotDrive, leftFollower);
    SendableRegistry.addChild(m_robotDrive, rightLeader);
    SendableRegistry.addChild(m_robotDrive, rightFollower);
   
  }

  @Override
  public void teleopPeriodic() {
    // speed damper
    double speedFactor = 0.7;

    // Use the joystick Y axis for forward movement, X axis for lateral, movement, and Z axis for rotation.
    //m_robotDrive.driveCartesian(0, 0.1, 0);

    

    //m_robotDrive.driveCartesian(-m_stick.getY() * speedFactor, m_stick.getX() * speedFactor, -m_stick.getZ() * speedFactor, Rotation2d.kZero);
    
    System.out.println(ArmMotor.getEncoder().getPosition());
    System.out.println(ClawMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Arm position", ClawMotor.getEncoder().getPosition());

     if (m_stick.getRawButton(3)) {
      wristMotor.set(0.1);
     } else if (m_stick.getRawButton(4) ) {
     wristMotor.set(-0.1);
      } else {
     wristMotor.set(0);
     }
    
     if (m_stick.getRawButton(5)) {
        ArmMotor.set(0.24);
        SaveValue = ArmMotor.getEncoder().getPosition();
        ButtonPushed= true;
        StayMode=false;
    } else if (m_stick.getRawButton(6)) {
      ArmMotor.set(-0.18);
      SaveValue = ArmMotor.getEncoder().getPosition();
      ButtonPushed= true;
      StayMode=false;
    } else {
      ArmMotor.set(0.0);
      if (ButtonPushed) {
        ButtonPushed = false;
        StayMode = true;
        SaveValue = ArmMotor.getEncoder().getPosition();

      } else if (StayMode) {
        if((ArmMotor.getEncoder().getPosition() - SaveValue) < 0)  {
          ArmMotor.set(0.05);
        } else if((ArmMotor.getEncoder().getPosition() - SaveValue) > 0) {
          ArmMotor.set(-0.05);
        } else {
          ArmMotor.set(0);
        }
      }
    }


      if (m_stick.getRawButton(1)) {
      ClawMotor.set(0.5);
  } else if (m_stick.getRawButton(2) || m_stick.getRawButton(11)) {
    ClawMotor.set(-0.5);
  } else {
    ClawMotor.set(0);
  }



   /***  if (m_stick.getRawButton(1)) {
      wristMotor.set(0.05);
    } else {
      wristMotor.set(0);
    }
  
    
    if (m_stick.getRawButton(2)) {
      wristMotor.set(0.05);
    } else {
      wristMotor.set(0);
    }
   **/
    /*********
    // input: theta and power
    double y = m_stick.getY();
    double x = m_stick.getX();
    double turn = m_stick.getZ();

    double theta = Math.atan2 (y,x);
    double power = Math.hypot (x,y);

    double sin = Math.sin (theta - Math.PI/4);
    double cos = Math.cos (theta - Math.PI/4);
    double max = Math.max (Math.abs (sin), Math.abs (cos));

    double leftLeaderSpeed = power * cos/max + turn;
    double rightLeaderSpeed = power * sin/max - turn;
    double leftFollowerSpeed = power * sin/max + turn;
    double rightFollowerSpeed = power * cos/max - turn;

    leftLeader.set(leftLeaderSpeed * speedFactor);
    rightLeader.set(rightLeaderSpeed * speedFactor);
    leftFollower.set(leftFollowerSpeed * speedFactor);
    rightFollower.set(rightFollowerSpeed * speedFactor);

    if ((power + Math.abs (turn)) > 1)
    {
      leftLeaderSpeed /= power + turn;
      rightLeaderSpeed /= power + turn;
      leftFollowerSpeed /= power + turn;
      rightFollowerSpeed /= power + turn;
      leftLeader.set(leftLeaderSpeed * speedFactor);
      rightLeader.set(rightLeaderSpeed * speedFactor);
      leftFollower.set(leftFollowerSpeed * speedFactor);
      rightFollower.set(rightFollowerSpeed * speedFactor);
    }
   
   **********/
  /***
      leftLeader.set(-0.1);
      rightLeader.set(0);
      leftFollower.set(0.1);
      rightFollower.set(0);
      ***/
  }

  @Override
  public void autonomousInit() {
      mTimer.reset();
      mTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
      if (mTimer.get() < 3.0) {
          // Drive forward with mecanum drive (0.0 is no strafe, 0.5 is half speed forward, 0.0 is no rotation)
          m_robotDrive.driveCartesian(0.2, 0.0, 0.0);
      } else {
          // Stop the robot after 4 seconds
          m_robotDrive.driveCartesian(0.0, 0.0, 0.0);
          mTimer.stop();
      }
  }
}