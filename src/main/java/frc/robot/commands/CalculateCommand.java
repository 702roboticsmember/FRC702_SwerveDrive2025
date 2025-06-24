package frc.robot.commands;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ReleaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CalculateCommand extends Command{
    private ShooterSubsystem s_ShooterSubsystem;
    private ReleaseSubsystem r_ReleaseSubsystem;
    private double distance;
    private double angle = Constants.ShootSubsystem.ShootAngle;
    private double height = Constants.ShootSubsystem.ShootHeight;//feet

    public CalculateCommand(ShooterSubsystem s_ShooterSubsystem, ReleaseSubsystem r_ReleaseSubsystem, double distance){
        this.s_ShooterSubsystem = s_ShooterSubsystem;
        this.r_ReleaseSubsystem = r_ReleaseSubsystem;
        addRequirements(s_ShooterSubsystem);
        this.distance = distance;//feet
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      // 0 = y + sin(45)vt + 0.5at^2
      
      //y = t(sin(45)v + at)
      //y = distancetan(45) + 0.5a(d/cos(45)v)^2
      //t = distance/cos(45)v

      double Velocity = Math.sqrt((16 * distance * distance)/((Math.cos(angle/180 * Math.PI) * Math.cos(angle/180 * Math.PI) ) * (height - (distance * Math.tan(angle/180 * Math.PI)))));
      
      new ShootCommand(s_ShooterSubsystem, r_ReleaseSubsystem, Velocity);
    }

    @Override
    public void end(boolean interrupted) {
        s_ShooterSubsystem.setVelocity(0);
    }

    @Override
    public boolean isFinished() {
    return false;
    }

}
