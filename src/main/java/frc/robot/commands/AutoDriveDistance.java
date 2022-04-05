// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;


public class AutoDriveDistance extends CommandBase {
  private final DriveTrain m_driveTrain;
  private double m_speed;
  private double m_setpoint;
  /** Creates a new AutoDriveDistance. */
  public AutoDriveDistance(DriveTrain m_driveTrain, double speed, double setpoint) {
    this.m_driveTrain = m_driveTrain;
    m_speed = speed;
    m_setpoint = setpoint;
    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // m_driveTrain.resetEncoder();
    
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      m_driveTrain.my_TankDrive(m_speed, m_speed);
       System.out.println(m_driveTrain.get_Right_Encoder_inch());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.my_TankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;

    return Math.abs(m_driveTrain.get_Right_Encoder_inch()) >= m_setpoint;

  }
}
