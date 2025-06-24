package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public TalonFXConfiguration turretConfigs = new TalonFXConfiguration();
    public TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
    public TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();



    public CTREConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.CANCODER_INVERT;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.ANGLE_MOTOR_INVERT;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.ANGLE_NEUTRAL_MODE;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.ANGLE_GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.ANGLE_ENABLE_STATOR_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.ANGLE_STATOR_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.ANGLE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.ANGLE_CURRENT_LIMIT;
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.ANGLE_CURRENT_THRESHOLD; //these variables got deleted, didnt find a replacement
        // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.ANGLE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.ANGLE_PID.kP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.ANGLE_PID.kI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.ANGLE_PID.kD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.DRIVE_MOTOR_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.DRIVE_NEUTRAL_MODE;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DRIVE_GEAR_RATIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.DRIVE_ENABLE_STATOR_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.DRIVE_STATOR_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.DRIVE_CURRENT_LIMIT;
        //swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.DRIVE_CURRENT_THRESHOLD; //these variables got deleted, didnt find a replacement
        //swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.DRIVE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.DRIVE_PID.kP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.DRIVE_PID.kI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.DRIVE_PID.kD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
    
        //turret configs
        
        var currentlimits = turretConfigs.CurrentLimits;
        currentlimits.StatorCurrentLimit = 0;
        currentlimits.StatorCurrentLimitEnable = false;
        currentlimits.SupplyCurrentLimit = 0;
        currentlimits.SupplyCurrentLimitEnable = false;

        var motoroutput = turretConfigs.MotorOutput;
        motoroutput.NeutralMode = NeutralModeValue.Brake;
        motoroutput.Inverted = InvertedValue.Clockwise_Positive;
        

        // set slot 0 gains
        var slot0Configs = turretConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
        
        var limits = turretConfigs.SoftwareLimitSwitch;
        limits.ForwardSoftLimitThreshold = 0;
        limits.ForwardSoftLimitEnable = false;
        limits.ReverseSoftLimitThreshold = 0;
        limits.ReverseSoftLimitEnable = false;

        // set Motion Magic Velocity settings
        var motionMagicConfigs = turretConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 4000 rps/s/s (0.1 seconds)
        turretConfigs.Feedback.SensorToMechanismRatio = 1/14;
        turretConfigs.ClosedLoopGeneral.ContinuousWrap = true;

   

        //shooter configs
        var s_currentlimits = shooterConfigs.CurrentLimits;
        s_currentlimits.StatorCurrentLimit = 0;
        s_currentlimits.StatorCurrentLimitEnable = false;
        s_currentlimits.SupplyCurrentLimit = 0;
        s_currentlimits.SupplyCurrentLimitEnable = false;

        var s_motoroutput = shooterConfigs.MotorOutput;
        s_motoroutput.NeutralMode = NeutralModeValue.Brake;
        s_motoroutput.Inverted = InvertedValue.Clockwise_Positive;

        var s_slot0Configs = shooterConfigs.Slot0;
        s_slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        s_slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        s_slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        s_slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        s_slot0Configs.kI = 0; // no output for integrated error
        s_slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var s_motionMagicConfigs = shooterConfigs.MotionMagic;
        s_motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        s_motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)


        //intake configs
        var i_currentlimits = intakeConfigs.CurrentLimits;
        i_currentlimits.StatorCurrentLimit = Constants.IntakeConstants.STATOR_CURRENT_LIMIT;
        i_currentlimits.StatorCurrentLimitEnable = Constants.IntakeConstants.ENABLE_STATOR_CURRENT_LIMIT;
        i_currentlimits.SupplyCurrentLimit = Constants.IntakeConstants.CURRENT_LIMIT;
        i_currentlimits.SupplyCurrentLimitEnable = Constants.IntakeConstants.ENABLE_CURRENT_LIMIT;

        var i_motoroutput = intakeConfigs.MotorOutput;
        i_motoroutput.NeutralMode = NeutralModeValue.Brake;
        i_motoroutput.Inverted = InvertedValue.Clockwise_Positive;

    }
}