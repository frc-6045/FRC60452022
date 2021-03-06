// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class DriveTrainInvert extends InstantCommand {
  private final DriveTrain m_DriveTrain;
  private boolean m_invert;
  private boolean setInvert;

  public DriveTrainInvert(DriveTrain subsystem, boolean invert) {
    m_DriveTrain = subsystem;
    m_invert = invert;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveTrain.my_InvertDrive(m_invert);
  }
}
