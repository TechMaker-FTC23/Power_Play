package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group="drive")
public class TechmakerTeleop extends LinearOpMode {
    public double elevator_position = 0;
    public double error = 0;
    public double kP = 0.005;
    public double elevatorVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor elevator;
        Servo intake1 = hardwareMap.get(Servo.class,"servoleft");
        Servo intake2 = hardwareMap.get(Servo.class,"servoright");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator = hardwareMap.get(DcMotor.class,"elevatormotor");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        while(!isStopRequested()){
            drive.setWeightedDrivePower(
                    new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x));
            if(gamepad1.a)
                elevator_position = 200;

            if(gamepad1.b)
                elevator_position = 1900;


            if(gamepad1.y)
                elevator_position = 3000;

            if(gamepad1.x)
                elevator_position = 4500;

            error = elevator_position-elevator.getCurrentPosition();

            elevatorVelocity = error * kP;

            elevator.setPower(elevatorVelocity);

            if(gamepad1.right_bumper) {
                intake2.setPosition(2);
                intake1.setPosition(0);
            }

            if(gamepad1.left_bumper) {
                intake2.setPosition(0);
                intake1.setPosition(2);
            }
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Elevador",elevator.getCurrentPosition());
            telemetry.addData("Elevador Velocity",elevatorVelocity);
            telemetry.addData("Elevador Error",error);


            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
