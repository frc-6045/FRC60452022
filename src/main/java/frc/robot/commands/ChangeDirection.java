// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ChangeDirection extends CommandBase {
  private final DriveTrain m_driveTrain;
  private Joystick leftJoy;
  private Joystick rightJoy;
  private Joystick arcJoy;
  
  /** Creates a new ChangeDirection. */
  public ChangeDirection(DriveTrain subsystem, Joystick leftJoy, Joystick rightJoy, Joystick arcJoy) {
   m_driveTrain = subsystem;
   this.leftJoy = leftJoy;
   this.rightJoy = rightJoy;
   this.arcJoy = arcJoy;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.DrivePrefrance == 0){
      m_driveTrain.driveTank(leftJoy.getY() * Constants.DriveSpeed, rightJoy.getY() * -Constants.DriveSpeed);
    }
    else{
      //m_driveTrain.driveTank(arcJoy.getX() * -Constants.DriveSpeed, arcJoy.getY() * Constants.DriveSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
