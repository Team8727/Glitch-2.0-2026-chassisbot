// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}
  public static boolean isChassisBot = false;
  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    try {
      for (NetworkInterface netIf : Collections.list(NetworkInterface.getNetworkInterfaces())) {
        byte[] mac = netIf.getHardwareAddress();
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < mac.length; i++) {
          sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
        }
        System.out.println(sb);
        if (sb.toString().equals("addr")) {
          isChassisBot = true;
        }
      }
    } catch (SocketException e) {
      e.printStackTrace();
    }

    RobotBase.startRobot(Robot::new);
  }
}
