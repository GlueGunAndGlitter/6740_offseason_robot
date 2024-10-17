// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class Led {
    static AddressableLED led = new AddressableLED(9);
    static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(30);
   
 public Led(){

    led.setLength(30);
    led.setData(ledBuffer);
    led.start();
}

 public static void colorRed(){
  for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 255, 0, 255);   
    }
    
 }

}
