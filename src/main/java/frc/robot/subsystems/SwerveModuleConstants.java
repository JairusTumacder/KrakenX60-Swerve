package frc.robot.subsystems;

public class SwerveModuleConstants{
  public final int driveMotorCANID;
  public final int turningMotorCANID;
  public final int canCoderID;
  public final boolean driveMotorInverted;
  public final boolean turningMotorInverted;
  public final double encOffset;

  public SwerveModuleConstants(int driveMotorCANID, int turningMotorCANID, int canCoderID, boolean driveMotorInverted, boolean turningMotorInverted, double encOffset){
    this.driveMotorCANID = driveMotorCANID;
    this.turningMotorCANID = turningMotorCANID;
    this.canCoderID = canCoderID;
    this.driveMotorInverted = driveMotorInverted;
    this.turningMotorInverted = turningMotorInverted;
    this.encOffset = encOffset;
  }

}
