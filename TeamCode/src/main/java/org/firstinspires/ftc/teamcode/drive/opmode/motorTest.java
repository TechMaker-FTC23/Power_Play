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

@TeleOp(group="drive")
public class motorTest extends LinearOpMode {
    public double timeoutMotor = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor motor;
        motor = hardwareMap.get(DcMotor.class,"elevatormotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initEncoder();
        waitForStart();

        while(!isStopRequested()) {
            if(gamepad1.a) {
                motor.setPower(1);
                timeoutMotor = getRuntime() + 5;
            }
            if(getRuntime()>timeoutMotor){
                motor.setPower(0);
            }
            telemetry.addData("Elevador", motor.getCurrentPosition());
            telemetry.update();
        }
    }


    public void initEncoder() {

    }
}
