// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.*;



public class TurretSubsystem extends SubsystemBase {
  private TalonSRX Motor = new TalonSRX(0);
  //private SparkMax Motor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed);
  private DutyCycleEncoder m_encoderFR = new DutyCycleEncoder(0, 4.0, 2.0);
  
  // private Spark LeftMotor = new Spark(Constants.CoralIntakeConstants.LeftMotorID);
  // private Spark RightMotor = new Spark(Constants.CoralIntakeConstants.RightMotorID);
 // private DigitalInput sensor = new DigitalInput(Constants.LIMIT_SWITCH_INTAKE);
  /** Creates a new ClimbSubsystem. */
   public TurretSubsystem() {
    

    //Motor.configure(Robot.CTRE_CONFIGS.turretConfig, Constants.TurretConstants.resetMode, Constants.TurretConstants.persistMode);
  // /** Creates a new ReleaseSubsystem. */
 }

  @Override
  public void periodic() {
    Motor.setSelectedSensorPosition(getAngle());
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    Motor.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putBoolean("hiiiiiii", true);
    
  }

  public Command run(DoubleSupplier input){
    
    return this.runEnd(() -> this.setSpeed(MathUtil.applyDeadband(input.getAsDouble(), 0.1)), () -> this.setSpeed(0.0));
  }

  public double getAngle() {
    //m_encoderFR.isConnected();
    return m_encoderFR.get();
    
  }
  
}
