// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ReleaseSubsystem extends SubsystemBase {

  private SparkMax Motor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed);
   // private Spark LeftMotor = new Spark(Constants.CoralIntakeConstants.LeftMotorID);
  // private Spark RightMotor = new Spark(Constants.CoralIntakeConstants.RightMotorID);
 // private DigitalInput sensor = new DigitalInput(Constants.LIMIT_SWITCH_INTAKE);
  /** Creates a new ClimbSubsystem. */
   public ReleaseSubsystem() {
   
    Motor.configure(Robot.CTRE_CONFIGS.releaseConfig, Constants.ReleaseConstants.resetMode, Constants.ReleaseConstants.persistMode);
  // /** Creates a new ReleaseSubsystem. */
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    Motor.set(speed);
    
  }

  public Command run(DoubleSupplier input){
    return this.runEnd(() -> this.setSpeed(MathUtil.applyDeadband(input.getAsDouble(), 0.1)), () -> this.setSpeed(0.0));
  }
}
