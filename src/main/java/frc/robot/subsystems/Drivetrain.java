// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.lang.Math;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  //Initalize motor controllers
  private final WPI_TalonFX m_leftLead = new WPI_TalonFX(Constants.leftFalconLeadId);
  private final WPI_TalonFX m_leftFollow = new WPI_TalonFX(Constants.leftFalconFollowId);
  private final WPI_TalonFX m_rightLead = new WPI_TalonFX(Constants.rightFalconLeadId);
  private final WPI_TalonFX m_rightFollow = new WPI_TalonFX(Constants.rightFalconFollowId);

  //Create motor controller groups
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftLead,m_leftFollow); 
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightLead,m_rightFollow);

  //Create Tank drive
  public final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  public Drivetrain() {
    m_left.setInverted(Constants.leftInvert);
    m_right.setInverted(Constants.rightInvert);

  }

  public static double NonLinear(double input){

    if(Math.abs(input)<.7){
      return input;
    }

    else return Math.pow(input,3);
  }



}