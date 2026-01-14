// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Armazenamento extends SubsystemBase {
  
  private SparkMax motorArmazenamento = new SparkMax(19, MotorType.kBrushless);

  public Armazenamento() {
    SparkMaxConfig configArmazenamento = new SparkMaxConfig();

    configArmazenamento.idleMode(SparkBaseConfig.IdleMode.kCoast);

    configArmazenamento.openLoopRampRate(0);

    motorArmazenamento.configure(
      configArmazenamento,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    );

  }

  public void setMotorArmazenamento(double Speed){
    motorArmazenamento.set(Speed);
  }

  public void StopMotorArmazenamento(){
    motorArmazenamento.set(0);
  }

  @Override
  public void periodic() {
  }
}
