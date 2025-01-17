package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.jni.AHRSJNI;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Our main drive subsystem
 */
public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveModules;
    public AHRS gyro;
    public  RobotConfig config;

    public Swerve() {
        gyro = new AHRS( NavXComType.kMXP_SPI);
        gyro.reset();
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        swerveModules = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        
    
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.KINEMATICS, getGyroYaw(), getModulePositions());
        
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            Constants.Swerve.PATHPLANNER_FOLLOWER_CONFIG,
            config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                      return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                  },
                  this // Reference to this subsystem to set requirements
          );
    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
        var swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i], false);
        }
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    private void resetPose(Pose2d startingPosition) {
        swerveOdometry.resetPosition(
                new Rotation2d(Math.toRadians(gyro.getAngle())),
                this.getModulePositions(),
                startingPosition);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return (Constants.Swerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getAcc() {
        return gyro.getAccelFullScaleRangeG();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putNumber("Acc",this.getAcc());
        for (SwerveModule mod : swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}