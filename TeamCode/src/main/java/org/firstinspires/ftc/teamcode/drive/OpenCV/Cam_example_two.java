/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.OpenCV;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.METERS_PER_PULSE;

import java.lang.Math;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous
public class Cam_example_two extends LinearOpMode
{
    DcMotor xMotor;
    DcMotor yMotor;
    Encoder xEncoder;
    Encoder yEncoder;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.037;

    int left = 18;
    int right = 17;
    int middle = 19;

    AprilTagDetection tagOfInterest = null;
    public static final int x1 = 115, x2 = 17, x3 = -7;
    public static final int y1 = -24, yLeft = -33, yMiddle = 30, yRight = 80;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor elevator;
        CRServo intake1 = hardwareMap.get(CRServo.class, "servoleft");
        CRServo intake2 = hardwareMap.get(CRServo.class, "servoright");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator = hardwareMap.get(DcMotor.class, "elevatormotor");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initEncoder();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == left || tag.id == middle || tag.id == right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        resetEncoder();
        drive.setPoseEstimate(new Pose2d(0,0,0));
        Pose2d path = new Pose2d(0.4, 0, 0);
        drive.setWeightedDrivePower(path);
        while (getXCentimeter()<x1)
        {
            drive.setWeightedDrivePower(path);
            drive.update();

        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        sleep(100);
        path = new Pose2d(0,-0.4,0);
        drive.setWeightedDrivePower(path);
        while(mod(getYCentimeters())<mod(y1))
        {
            drive.setWeightedDrivePower(new Pose2d(0,-0.4,0));
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        drive.update();
        while(elevator.getCurrentPosition()<3800){
            elevator.setPower(0.7);
        }
        elevator.setPower(0.02);
        path = new Pose2d(0.4,0,0);
        drive.setWeightedDrivePower(path);
        while(getXCentimeter()<(x1+x2)){
            drive.setWeightedDrivePower(new Pose2d(0.4,0,0));
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        drive.update();
        intake1.setPower(1);
        intake2.setPower(-1);
        sleep(2000);
        intake1.setPower(0);
        intake2.setPower(0);
        path = new Pose2d(-0.4,0,0);
        drive.setWeightedDrivePower(path);
        drive.update();
        while (getXCentimeter()>(x1+x2+x3)){
            elevator.setPower(0);
            drive.setWeightedDrivePower(new Pose2d(-0.4,0,0));
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        drive.update();
        elevator.setPower(-0.7);
        while(elevator.getCurrentPosition()>10)
        {
            elevator.setPower(-0.7);

        }
        elevator.setPower(0);

        //codigo para ir para esquerda colocar o cone

        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
            if (tagOfInterest.id == left) {
                telemetry.addData("Direction", "left");
                telemetry.update();
                drive.setWeightedDrivePower(new Pose2d(0,-0.4,0));
                while(getYCentimeters()>y1+yLeft)
                {
                    drive.setWeightedDrivePower(new Pose2d(0,-0.4,0));
                    drive.update();
                }

                drive.setWeightedDrivePower(new Pose2d(0,0,0));



            } else if ( tagOfInterest.id == middle) {
                telemetry.addData("Direction", "front");
                telemetry.update();
                drive.setWeightedDrivePower(new Pose2d(0,0.4,0));
                while(getYCentimeters()<y1+yMiddle)
                {
                    drive.setWeightedDrivePower(new Pose2d(0,0.4,0));
                    drive.update();
                }

                drive.setWeightedDrivePower(new Pose2d(0,0,0));

            } else if (tagOfInterest.id == right) {
                telemetry.addData("Direction", "right");
                telemetry.update();
                drive.setWeightedDrivePower(new Pose2d(0,0.4,0));
                while(getYCentimeters()<y1+yRight)
                {
                    drive.setWeightedDrivePower(new Pose2d(0,0.4,0));
                }
            }
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */

        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        drive.update();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public void initEncoder(){
        xMotor = hardwareMap.get(DcMotor.class,"xEncoder");
        yMotor = hardwareMap.get(DcMotor.class,"yEncoder");
        xEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,"xEncoder"));
        yEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,"yEncoder"));
        resetEncoder();
    }
    public void resetEncoder(){
        xMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double getXCentimeter(){
        return xEncoder.getCurrentPosition()*METERS_PER_PULSE;

    }
    public double getYCentimeters(){
        return yEncoder.getCurrentPosition()*METERS_PER_PULSE;

    }
    public int mod(int value){
        if(value<0)
            return(value*-1);
        return value;
    }
    public double mod(double value){
        if(value<0)
            return(value*-1.0);
        return value;
    }
}