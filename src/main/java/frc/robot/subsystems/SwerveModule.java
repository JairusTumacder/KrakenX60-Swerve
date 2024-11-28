package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule{
  public int moduleID;

  private TalonFX driveMotor;
  private CANSparkMax turningMotor;
  private CANcoder absoluteEncoder;
  private PIDController turningPIDController;
  private RelativeEncoder turningEncoder;
  
  private double encOffset;

  public SwerveModule(int moduleID, SwerveModuleConstants moduleConstants){
    this.moduleID = moduleID;
    encOffset = moduleConstants.encOffset;

    driveMotor = new TalonFX(moduleConstants.driveMotorCANID);
    turningMotor = new CANSparkMax(moduleConstants.turningMotorCANID, MotorType.kBrushless);
    absoluteEncoder = new CANcoder(moduleConstants.canCoderID);

    turningEncoder = turningMotor.getEncoder();

    turningMotor.restoreFactoryDefaults();

    driveMotor.setInverted(moduleConstants.driveMotorInverted);
    turningMotor.setInverted(moduleConstants.turningMotorInverted);

    absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf));
    absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    turningPIDController = new PIDController(SwerveConstants.kP, SwerveConstants.kI, SwerveConstants.kD);
    turningPIDController.enableContinuousInput(-180, 180);
  }

  public double getDriveVelocity(){
    return driveMotor.getVelocity().getValue() * SwerveConstants.kVelocityConversionFactor;
  }

  public double getDrivePosition(){
    return driveMotor.getPosition().getValue() * SwerveConstants.kPositionConversionFactor;
  }

  public double getAbsoluteDegrees(){
    return (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360) - encOffset;
  }

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(turningEncoder.getPosition());
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteDegrees()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAbsoluteDegrees()));
  }

  public void setState(SwerveModuleState desiredState){
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

    double turningOutput = turningPIDController.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

    turningMotor.set(turningOutput);
    driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.kMaxSpeed * SwerveConstants.kVoltage);
  }

  public void setAngle(SwerveModuleState desiredState){
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

    double turningOutput = turningPIDController.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

    turningMotor.set(turningOutput);
    driveMotor.set(0);
  }

  public void stop(){
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void print(){
    SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] Absolute Encoder Degrees", getAbsoluteDegrees());
    SmartDashboard.putNumber("S[" + moduleID + "] Drive Encoder", getDrivePosition());
    SmartDashboard.putNumber("S[" + moduleID + "] Turning Encoder", turningEncoder.getPosition());
  }
}
