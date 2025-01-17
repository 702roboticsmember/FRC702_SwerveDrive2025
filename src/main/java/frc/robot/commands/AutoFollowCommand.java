// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class AutoFollowCommand extends Command {
  boolean interrupted;

  private PIDController TranslatePID = new PIDController(

      Constants.AutoFollowConstants.kP,
      Constants.AutoFollowConstants.kI,
      Constants.AutoFollowConstants.kD);

  private PIDController RotatePID = new PIDController(
      Constants.AutoAimConstants.kP,
      Constants.AutoAimConstants.kI,
      Constants.AutoAimConstants.kD);
  private PIDController StrafePID = new PIDController(

      Constants.AutoFollowConstants.kP,
      Constants.AutoFollowConstants.kI,
      Constants.AutoFollowConstants.kD);
  
  
  DoubleSupplier TX;
  DoubleSupplier TZ;
  DoubleSupplier RY;
  BooleanSupplier tv;
  Swerve s_Swerve;
  Rotation2d headingprev;
  final double x;
  final double z;
  final double ry;
  

  /** Creates a new AutoAim. */
  public AutoFollowCommand(DoubleSupplier TX, DoubleSupplier TZ,DoubleSupplier RY, BooleanSupplier tv, double x, double z, double ry, Swerve s_Swerve) {
    this.TX = TX;
    this.TZ = TZ;
    this.RY = RY;
    this.tv = tv;
    this.s_Swerve = s_Swerve;
    this.headingprev = s_Swerve.getHeading();
    this.x = x;
    this.z = z;
    this.ry = ry;

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.robotCentric = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.setHeading(new Rotation2d(RY.getAsDouble() + 180));
    TranslatePID.setSetpoint(x);
    TranslatePID.setTolerance(.22);
    StrafePID.setSetpoint(z);
    StrafePID.setTolerance(.22);
    RotatePID.setSetpoint(ry);
    RotatePID.setTolerance(7);
    

    double x = TX.getAsDouble();
    boolean Target = tv.getAsBoolean();
    double value = TranslatePID.calculate(x);
    double result = Math.copySign(Math.abs(value) + 0.0955, value); 
    double Tranlate = (Target ? MathUtil.clamp(result, -0.87, 0.87) : 0);
    SmartDashboard.putNumber("TPID", value);
    SmartDashboard.putNumber("TTX", x);

    double z = TZ.getAsDouble();
    double value1 = StrafePID.calculate(z);
    double result1 = Math.copySign(Math.abs(value1) + 0.0955, value1); 
    double Strafe = (Target ? MathUtil.clamp(result1, -0.87, 0.87) : 0);
    SmartDashboard.putNumber("SPID", value1);
    SmartDashboard.putNumber("STZ", z);

    double a = RY.getAsDouble();
    double value2 = RotatePID.calculate(a);
    double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    double Rotate = (Target ? MathUtil.clamp(result2, -0.57, 0.57) : 0);
    SmartDashboard.putNumber("RRY", a);
    s_Swerve.drive(
                new Translation2d(-Tranlate, -Strafe).times(Constants.Swerve.MAX_SPEED),
                Rotate * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !true,
                false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.setHeading(headingprev);
   RobotContainer.robotCentric = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
