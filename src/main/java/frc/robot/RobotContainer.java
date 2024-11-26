package frc.robot;

import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
  
  SwerveDriveSubsystem s_Subsystem = new SwerveDriveSubsystem();

  XboxController xboxController = new XboxController(0);

  public RobotContainer() {
    s_Subsystem.setDefaultCommand(new DriveCommand(s_Subsystem, () -> xboxController.getLeftY() * 0.3, () -> xboxController.getLeftX() * 0.3, () -> xboxController.getRightX() * 0.6, true));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(xboxController, XboxController.Button.kA.value).onTrue(new InstantCommand(s_Subsystem::resetPigeon2));
  }

 
  public Command getAutonomousCommand() {
    return null;
  }
}
