// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSub extends SubsystemBase {
 private SparkMax climbMotor = new SparkMax(20,MotorType.kBrushless);
 private SparkMax climbMotor2 = new SparkMax(21,MotorType.kBrushless);
 public RelativeEncoder climbEncoder = climbMotor.getEncoder();
 
  public ClimbSub() {
    climbEncoder.setPosition(0);
    // ------ configure motor 1 (leader) ------//
    final SparkMaxConfig Kbrake = new SparkMaxConfig();
    final SparkMaxConfig Kbrake2 = new SparkMaxConfig();
      Kbrake.idleMode(SparkBaseConfig.IdleMode.kBrake);
      Kbrake.openLoopRampRate(0);

    // ------ configure motor 2 (follower) ------//
      Kbrake2.idleMode(SparkBaseConfig.IdleMode.kBrake);
      Kbrake2.openLoopRampRate(0);
      Kbrake2.follow(climbMotor, true);
    // ------ apply Config ------//
      climbMotor.configure(Kbrake, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      climbMotor2.configure(Kbrake2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
     

  }
     
    public void setMotor(double speed){
      climbMotor.set(speed);
    }

    public void MANUAL(){
      if (!atUpperLimit()) setMotor(-0.5);
      else STOP();
    }
    public void STOP(){
      climbMotor.set(0);
      
    }
    public void NegMANUAL(){
    if (!atLowerLimit()) setMotor(0.5);
    else STOP();
    }

    public boolean atUpperLimit() {
      return climbEncoder.getPosition() <= -0.5;
  }
  
  public boolean atLowerLimit() {
      return climbEncoder.getPosition() >= 0.0;
  }

    public double getPosition() {
      return climbEncoder.getPosition();
    }

    public double getVelocity() {
      return climbEncoder.getVelocity();
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ClimbMotors OutPut", climbMotor.get());
  }
}
 