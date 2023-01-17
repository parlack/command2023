

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveTrainCmd;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

public final static DriveSubsystem driveSubsystem =new DriveSubsystem();


public Joystick XboxController_main= new Joystick(0);


  public RobotContainer() {


    Controles1persona();


  }

  private void Controles1persona() {

    driveSubsystem.setDefaultCommand(new DriveTrainCmd(driveSubsystem, () -> XboxController_main.getRawAxis(3) -
    XboxController_main.getRawAxis(2), () -> XboxController_main.getRawAxis(0)));
    

    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
