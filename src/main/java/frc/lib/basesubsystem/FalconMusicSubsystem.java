// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.basesubsystem;
import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FalconMusicSubsystem extends SubsystemBase {
  public static class Constants {
    public TalonFX[] falcons;
  }


  public Orchestra orchestra;

  public FalconMusicSubsystem(Constants constants) {
    // ArrayList for instruments (Falcons). If more or less than four are used, add them to the ArrayList.
    ArrayList<TalonFX> instruments = new ArrayList<>(Arrays.asList(constants.falcons));
    // Orchestra
    orchestra = new Orchestra(instruments);
  }

  @Override
  public void periodic() {}

  public void loadMusic(String filename) {
    // Loads music from songsList. Required before using play()
    orchestra.loadMusic(filename);
  }

  public void play() {
    // Plays music. A track needs to be loaded before running this (loadMusic()).
    orchestra.play();
  }
  
  public void pause() {
    // Runs pause() if music is playing.
    if (orchestra.isPlaying()) {
      orchestra.pause();
    }
  }

  public void stop() {
    // Runs stop() if music is playing.
    if (orchestra.isPlaying()) {
      orchestra.stop();
    }
  }
}