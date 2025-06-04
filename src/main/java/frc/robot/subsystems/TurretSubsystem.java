// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX TurretMotor = new TalonFX(Constants.ShootSubsystem.ShootMotorID);

    final DynamicMotionMagicVoltage m_request =
   new DynamicMotionMagicVoltage(0, 80, 400, 0);

    public TurretSubsystem(){
        

        TurretMotor.getConfigurator().apply(Robot.CTRE_CONFIGS.turretConfigs);
    }
    public void set(double value) {
        TurretMotor.setVoltage(value * 12);
    }
    public void setMode(NeutralModeValue mode) {
        TurretMotor.setNeutralMode(mode);
    }

    public void setVelocity(double value){
        TurretMotor.setControl(m_request.withVelocity(value));
    }

    public Command runCmd(double value) {
        return this.run(() -> this.set(value));
    }
}
