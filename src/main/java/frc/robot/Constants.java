package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.SwerveModuleConstants;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SwerveConstants{

    public static final double kWheelDiameter = 4 * 2.5 / 100;
    public static final double kTrackWidth = 0.635;
    public static final double kWheelBase = 0.635;

    public static final double kGearRatio = 8.14 / 1;

    public static final double kVoltage = 7.2;

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
      //FL
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      //BL
      new Translation2d(-kWheelBase/2, kTrackWidth / 2),
      //FR
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      //BR
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    public static class FrontLeft{
      public static final int kDriveID = 1;
      public static final int kTurningID = 5;
      public static final int kCANID = 9;
      public static final boolean kDriveInverted = false;
      public static final boolean kTurningInverted = true;
      public static final double kEncOffset = -102;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveID, kTurningID, kCANID, kDriveInverted, kTurningInverted, kEncOffset);
    }

    public static class BackLeft{
      public static final int kDriveID = 2;
      public static final int kTurningID = 6;
      public static final int kCANID = 10;
      public static final boolean kDriveInverted = false;
      public static final boolean kTurningInverted = true;
      public static final double kEncOffset = 84;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveID, kTurningID, kCANID, kDriveInverted, kTurningInverted, kEncOffset);
    }

    public static class FrontRight{
      public static final int kDriveID = 4;
      public static final int kTurningID = 8;
      public static final int kCANID = 12;
      public static final boolean kDriveInverted = false;
      public static final boolean kTurningInverted = true;
      public static final double kEncOffset = 153;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveID, kTurningID, kCANID, kDriveInverted, kTurningInverted, kEncOffset);
    }

    public static class BackRight{
      public static final int kDriveID = 3;
      public static final int kTurningID = 7;
      public static final int kCANID = 11;
      public static final boolean kDriveInverted = false;
      public static final boolean kTurningInverted = true;
      public static final double kEncOffset = 158;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveID, kTurningID, kCANID, kDriveInverted, kTurningInverted, kEncOffset);
    }

    public static final double kPositionConversionFactor = kGearRatio * Math.PI * kWheelDiameter;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static final double kP = 0.0048;
    public static final double kI = 0.0002;
    public static final double kD = 0.0001;

    public static final double kMaxSpeed = 3.6576;

  }
}
