package frc.robot.commands;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ReleaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    private ShooterSubsystem s_ShooterSubsystem;
    private ReleaseSubsystem r_ReleaseSubsystem;
    private double Velocity;
    public ShootCommand(ShooterSubsystem s_ShooterSubsystem, ReleaseSubsystem r_ReleaseSubsystem, double Velocity){
        addRequirements(s_ShooterSubsystem);
        this.Velocity = Velocity;
        this.r_ReleaseSubsystem = r_ReleaseSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        new SequentialCommandGroup(
            new InstantCommand(() -> s_ShooterSubsystem.setVelocity(Velocity)),
            new InstantCommand(() -> r_ReleaseSubsystem.setSpeed(5)),
            new WaitCommand(2),
            new InstantCommand(() -> r_ReleaseSubsystem.setSpeed(0)),
            new InstantCommand(() -> s_ShooterSubsystem.setVelocity(0))            
        );
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
