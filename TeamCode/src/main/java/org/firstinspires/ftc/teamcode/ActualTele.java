package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;
import com.vuforia.CameraDevice;



@TeleOp(name = "ActualTeleWontWork")
public class ActualTele extends LinearOpMode {
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    //DcMotor shooter;
    //DcMotor middleWheel;
    //DcMotor intake;
    //DcMotor wobble;
    ModernRoboticsI2cRangeSensor lefty;
    ModernRoboticsI2cRangeSensor righty;
    ModernRoboticsI2cRangeSensor looft;
    ModernRoboticsI2cRangeSensor rooft;
    Servo grip;

    @Override
    public void runOpMode() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        //   lefty = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "lefty");
        //   righty = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "righty");
        //   looft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "looft");
        //   rooft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rooft");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        waitForStart();


        while(opModeIsActive())
        {
            // telemetry.addData("FL:", lefty.rawUltrasonic());
            // telemetry.addData("FR:", righty.rawUltrasonic());
            // telemetry.addData("L:", looft.rawUltrasonic());
            // telemetry.addData("R:", rooft.rawUltrasonic());


            double drive = gamepad1.left_stick_y * 0.65;
            double strafe = -gamepad1.left_stick_x * 0.5;
            double spin = gamepad1.right_stick_x * 0.4;
            double lift = gamepad1.right_stick_y * 0.9;
            double up = -gamepad2.left_stick_y* 0.7;



            frontLeft.setPower(drive + strafe - spin);
            frontRight.setPower(drive - strafe + spin);
            backLeft.setPower(drive - strafe - spin);
            backRight.setPower(drive + strafe + spin);
            final int robotRadius = 9; // inches



            if(gamepad1.dpad_right)
            {
                frontLeft.setPower(-.50);
                frontRight.setPower(.50);
                backLeft.setPower(.50);
                backRight.setPower(-.50);
            }
            if(gamepad1.dpad_left)
            {
                frontLeft.setPower(.50);
                frontRight.setPower(-.50);
                backLeft.setPower(-.50);
                backRight.setPower(.50);
            }
            if(gamepad1.dpad_up)
            {
                frontLeft.setPower(.50);
                frontRight.setPower(.50);
                backLeft.setPower(.50);
                backRight.setPower(.50);
            }
            if(gamepad1.dpad_down)
            {
                frontLeft.setPower(-.50);
                frontRight.setPower(-.50);
                backLeft.setPower(-.50);
                backRight.setPower(-.50);
            }








        }

        while(opModeIsActive())
        {
            //telemetry.addData("RPM:",);
            //telemetry.update();
        }

    }


}





