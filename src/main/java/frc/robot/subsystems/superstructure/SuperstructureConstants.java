// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

public class SuperstructureConstants {

    public static final int feederCanId = 51;
    public static final double feederMotorReduction = 1.0;
    public static final int feederCurrentLimit = 60;

    public static final int intakeLauncherCanId = 50;
    public static final double intakeLauncherMotorReduction = 1.0;
    public static final int intakeLauncherCurrentLimit = 60;

    public static final double intakingFeederVoltage = -12.0;
    public static final double intakingLauncherVelocity = -550;
    public static final double intakingIntakeVoltage = 10.0;
    public static final double launchingFeederVoltage = 12.0;
    public static final double launchingLauncherVelocity = -410;
    public static final double spinUpSeconds = 1.0;

    public static final double kLauncherP = 9e-3;
    public static final double kLauncherI = 4e-6;
    public static final double kLauncherD = 5e-3;
}
