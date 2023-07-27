package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.METERS_PER_PULSE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(group="drive")
public class TechmakerTeleop extends LinearOpMode {
    public double elevator_position = 0;
    public double error = 0;
    public double kP = 0.001;
    public double elevatorVelocity = 0;
    public double mecanumVelocity = 0;
    public boolean closeIntake = false;
    public double timeoutIntake = 0;
    public boolean down = false;
    DcMotor xMotor;
    DcMotor yMotor;
    Encoder xEncoder;
    Encoder yEncoder;
    DistanceSensor distance;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor elevator;
        DcMotor ledPower = hardwareMap.get(DcMotor.class,"ledPower");

        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        double Distance = distance.getDistance(DistanceUnit.MM);
        //distance sensor

        CRServo intake1 = hardwareMap.get(CRServo.class,"servoleft");
        CRServo intake2 = hardwareMap.get(CRServo.class,"servoright");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator = hardwareMap.get(DcMotor.class,"elevatormotor");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initEncoder();
        ledPower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ledPower.setPower(0);

        waitForStart();


        while(!isStopRequested()){

            mecanumVelocity = (5500-elevator.getCurrentPosition())/4500.0;
            drive.setWeightedDrivePower(
                    new Pose2d(-gamepad1.left_stick_y*mecanumVelocity,-gamepad1.left_stick_x*mecanumVelocity,-gamepad1.right_stick_x*mecanumVelocity));


            if(gamepad1.left_trigger>0.5){
                down = true;
            }
            else{
                down = false;
            }
            if(gamepad1.a)
                elevator_position = 600;

            if(gamepad1.b)
                elevator_position = 1900;


            if(gamepad1.y)
                elevator_position = 3000;

            if(gamepad1.x)
                elevator_position = 4000;



            if(gamepad1.right_bumper) {
                elevator_position = 0;
                closeIntake = true;
            }
            if (elevator.getCurrentPosition() <= 350 && closeIntake) {
                intake2.setPower(1);
                intake1.setPower(-1);
                closeIntake = false;
                timeoutIntake = getRuntime()+1;

            }
            if(gamepad1.left_bumper) {
                intake2.setPower(-1);
                intake1.setPower(1);
                timeoutIntake = getRuntime()+1;

            }
            if(getRuntime()>timeoutIntake){
                intake2.setPower(0);
                intake1.setPower(0);
            }
            if(elevator.getCurrentPosition()>200)
            {
                ledPower.setPower(0.5);
            }
            else{
                ledPower.setPower(0);

            }
            if(gamepad1.back)
            {
                drive.setPoseEstimate(new Pose2d(0,0,0));
                resetEncoder();
                elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            if(down){
                elevator.setPower(-0.3);
                intake1.setPower(-2);
                intake2.setPower(2);
            }
            else {
                error = elevator_position - elevator.getCurrentPosition();

                elevatorVelocity = error * kP;

                elevator.setPower(elevatorVelocity);
            }
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Runtime", getRuntime());
            telemetry.addData("Elevador",elevator.getCurrentPosition());
            telemetry.addData("Elevador Velocity",elevatorVelocity);
            telemetry.addData("Elevador Error",error);
            telemetry.addData("Elevator Real",elevator.getPower());


            telemetry.addData("x", getXCentimeter());
            telemetry.addData("y", getYCentimeters());
            telemetry.addData("heading", getYCentimeters());
            telemetry.addData("Intake", intake1.getPower());
            telemetry.addData("Distance",Distance);
            telemetry.update();
        }
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
}