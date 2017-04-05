package org.usfirst.frc.team1923.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1923.robot.RobotMap;
import org.usfirst.frc.team1923.robot.utils.GripPipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class VisionSubsystem extends Subsystem {

    public double centerx, turn;
    public double[] widtharr;
    public double width;
    public double dist;

    public boolean found;

    private double sumx;
    private double sumw;
    private double[] def;

    public double contourX;

    public Ultrasonic frontSonar;

    Mat source = new Mat();
    Mat output = new Mat();
    CvSink cvSink;
    CvSource outputStream;
    GripPipeline pipe;
    Rect r;

    /**
     * Initializes CameraServer and NetworkTables
     */
    public VisionSubsystem() {
        try {
            // TODO: Implement Bounding Rectangle
            // TODO: IMplement Controller Vibration when Match time is getting
            // low
            // Start Camera Server
            // CameraServer.getInstance().addAxisCamera(RobotMap.CAMERA_IP);
            // Testing Drawing Bounding Rectangle Around Peg
            /*
             * CvSource output =
             * CameraServer.getInstance().putVideo("Annotated", 320, 240);
             */
            // TODO: Account for difference in areas of tape to change turn
            // value
            // TODO: Add ultrasonic sensors
            def = new double[0];
            frontSonar = new Ultrasonic(RobotMap.FRONT_SONAR_PING_PORT, RobotMap.FRONT_SONAR_ECHO_PORT, Unit.kInches);
            frontSonar.setEnabled(true);
            frontSonar.setAutomaticMode(true);
            dist = frontSonar.getRangeInches();
            found = false;

            AxisCamera camera = CameraServer.getInstance().addAxisCamera("Axis Cam", RobotMap.CAMERA_IP);
            camera.setResolution(320, 240);

            cvSink = CameraServer.getInstance().getVideo();
            outputStream = CameraServer.getInstance().putVideo("Processed", 320, 240);

            SmartDashboard.putNumber("Power", 0);

            pipe = new GripPipeline();

            refresh();
        } catch (Exception e) {
            System.out.println("Exception was thrown: " + e);
        }
    }

    public void refresh() {
        // TODO: Move refresh method to a seperate command to get automatic
        // multithreading
        try {
            // Process Image
            try {
                cvSink.grabFrameNoTimeout(source);

                pipe.process(source);
                sumx = 0;
                sumw = 0;
                if (!pipe.filterContoursOutput().isEmpty()) {
                    for (int i = 0; i < 2; i++) {
                        r = Imgproc.boundingRect(pipe.filterContoursOutput().get(i));
                        contourX = r.x + (r.width / 2);
                        sumx += contourX;
                        sumw += r.width;
                        // SmartDashboard.putNumber("Center X Pipeline: ",
                        // centerX);
                    }
                } else {
                    width = Integer.MAX_VALUE;
                    centerx = Integer.MIN_VALUE;
                }
                outputStream.putFrame(pipe.hslThresholdOutput());
                // outputStream.putFrame(output);
            } catch (UnsatisfiedLinkError b) {
                System.out.println("Error");
            }

            // Extrapolate Values (Turn, distance, etc.)
            dist = frontSonar.getRangeInches();
            // dist=16;

            if (!pipe.filterContoursOutput().isEmpty()) {
                width = sumw / pipe.filterContoursOutput().size();
                centerx = sumx / pipe.filterContoursOutput().size(); // centerx:
                                                                     // pixel
                                                                     // value of
                                                                     // middle
                                                                     // of peg
            }
            // Added 13 to make sure we dont hit the center of the gear
            turn = centerx - RobotMap.IMG_WIDTH / 2 + 4;
            turn /= RobotMap.TURN_CONSTANT;
            // Check Boundaries of turn
            if (turn < -1)
                turn = -1;
            else if (turn > 1)
                turn = 1; // TODO: Use PID to get to turn value and use an angle
                          // instead of turn (Using IMU)
            if (pipe.filterContoursOutput().isEmpty())
                turn = Integer.MIN_VALUE;

            // Logging
            SmartDashboard.putNumber("Center X: ", centerx);
            SmartDashboard.putNumber("Distance to target(Ultrasonic): ", dist);
            SmartDashboard.putNumber("Width: ", width);
            SmartDashboard.putNumber("Turn: ", turn);

        } catch (Exception e) {
            System.out.println("Exception was thrown: " + e);
        }

    }

    @Override
    public void initDefaultCommand() {
        // Uncomment for continuous Processing
        // setDefaultCommand(new VisionProcessing());
    }

    public Ultrasonic getUltrasonic() {
        return frontSonar;
    }

    public double getCenterX() {
        return centerx;
    }

    public double getDistance() {
        return dist;
    }

    public double getWidth() {
        return width;
    }

    public boolean isFound() {
        return centerx >= 0;
    }

    public double getTurn() {
        return turn;
    }

}
