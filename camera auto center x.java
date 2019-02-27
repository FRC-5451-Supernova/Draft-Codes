/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GripPipeline;

public class Robot extends TimedRobot {
  Thread m_visionThread;

  SpeedController chassis_left, chassis_right;

  XboxController stick;
  NetworkTableEntry target_0;

  GripPipeline pipe;

  Double[] default_array = new Double[2];
  

  @Override
  public void robotInit() {
    default_array[0] = default_array[1] = -1.;


    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("GRIP");
    target_0 = table.getEntry("myContoursReport/centerX");

    chassis_left = new Spark(1);
    chassis_right = new Spark(3);

    stick = new XboxController(0);

    pipe = new GripPipeline();

    m_visionThread = new Thread(() -> {
      // Get the UsbCamera from CameraServer
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      // Set the resolution
      camera.setResolution(320, 240);

      // Get a CvSink. This will capture Mats from the camera
      CvSink cvSink = CameraServer.getInstance().getVideo();
      // Setup a CvSource. This will send images back to the Dashboard
      CvSource outputStream
          = CameraServer.getInstance().putVideo("Rectangle", 320, 240);

      // Mats are very memory expensive. Lets reuse this Mat.
      Mat mat = new Mat();

      // This cannot be 'true'. The program will never exit if it is. This
      // lets the robot stop this thread when restarting robot code or
      // deploying.
      while (!Thread.interrupted()) {
        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat.  If there is an error notify the output.
        if (cvSink.grabFrame(mat) == 0) {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          // skip the rest of the current iteration
          continue;
        }
        // Put a rectangle on the image
        pipe.process(mat);
        // Give the output stream a new image to display
       //  outputStream.putFrame(mat);
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  public void move(double y, double z) {
    double throttle = 1;
    chassis_left.set(throttle * y - 0.4 * z);
    chassis_right.set(throttle * -y - 0.4 * z);
  }

  @Override
  public void teleopPeriodic() {
    
    if (stick.getAButton()) {

      Double[] xs = target_0.getDoubleArray(default_array);

      double center = 0;
      if (xs.length >= 2) {
        center = (xs[1] + xs[0]) / 2;
        center = (center - 160) / 160;
        System.out.println(center);
      }
      move(-stick.getY(Hand.kLeft), -1 * center);
      SmartDashboard.putNumber("turn throttle", center);
    } else {
      move(-stick.getY(Hand.kLeft), -stick.getX(Hand.kLeft));
    }
  }
}
