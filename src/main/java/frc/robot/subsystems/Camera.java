// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Camera extends SubsystemBase {

  /** Creates a new Camera. */
  public Camera() {
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
      camera.setResolution(Constants.Camera.WIDTH, Constants.Camera.HEIGHT);

      MjpegServer server = CameraServer.getInstance().addServer("cameraStream");

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Camera", Constants.Camera.WIDTH,
          Constants.Camera.HEIGHT);

      Mat source = new Mat();
      Mat output = new Mat();

      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          outputStream.notifyError(cvSink.getError());
          continue;
        }
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }

    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
