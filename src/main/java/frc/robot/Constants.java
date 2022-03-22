// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// Motor Constants
public static int frontRightDriveID = 1;
public static int backRightDriveID = 15;
public static int frontLeftDriveID = 3;
public static int backLeftDriveID = 2;
public static int intakeSpinID = 12;
public static int intakeRaiseID = 4;
public static int conveyorID = 14;
public static int dumpID = 13;
public static int liftID = 9;
//Drive Constants
public static int DrivePrefrance = 0;
public static double DriveSpeed = .6;
// Autonomous Constants
public static double autoDriveSpeed = .4;
public static double autoDriveTime = 10;
public static double autoScoreTime = 8;
public static double autoIntakeTime = 6;
//Intake Constants
public static double intakeSpinSpeed = .4;
public static double conveyorSpeed= .4;
public static double intakeRaiseSpeed = .4;
//Dump Constants
public static double dumpSpeed = .6;
//Climb Constants
public static final double liftMotorSpeed = 0.1;
//PID Constants
public static final double DrivePIDkd = 0;
public static final double DrivePIDki = 0;
public static final double DrivePIDkp = 0.0000001;
public static final double DrivePIDToleranceRPS = 0;
public static final double DrivePIDTargetPosition = 0;
public static final double DrivePIDTolerance = 0;

//Built In Encoder Constants
public static final double kWheelRadiusInches = 3;
public static final double kGearRatio = 5/3;
public static final double kCountsPerRev = 4096;
public static final double autoDriveDistance = 200000;


//Gyro Constants
public static double kAngleSetPoint = 0;
public static double kP = .008; //Proportional Turning constant

}
