package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class TestCameraOpMode extends OpMode
{
// We treat this like a singleton because there should onL    ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    //start adding the imu

    @Override
    public void init() {
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
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

        slamra.setPose(new Pose2d(-0 * 0.0254, 0 * 0.0254, new Rotation2d(0)));
        slamra.start();

    }

    @Override
    public void loop() {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);
        double drive = gamepad1.left_stick_y * 0.65;
        double strafe = -gamepad1.left_stick_x * 0.5;
        double spin = gamepad1.right_stick_x * 0.4;
        double lift = gamepad1.right_stick_y * 0.9;
        double up1 = -gamepad2.left_stick_y* 0.7;



        frontLeft.setPower(drive + strafe - spin);
        frontRight.setPower(drive - strafe + spin);
        backLeft.setPower(drive - strafe - spin);
        backRight.setPower(drive + strafe + spin);
        //final int robotRadius = 9; // inches



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
        if(gamepad1.a)
        {
            //Correction(0,0);
        }
        //while(opModeIsActive())
        //{
            telemetry.addData("X:",up.pose.getTranslation().getX() / 0.0254);
            telemetry.addData("Y:",up.pose.getTranslation().getY() / 0.0254);
            telemetry.update();
        //}
        //driveSon(.5,5);
        Correction(1,0);








    }

    @Override
    public void stop() {
        slamra.stop();
    }
    public void strafeLeft(double power, double inches) {
        int distance = (int) (57.580322 * inches - 21.521799);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(-distance);
        backLeft.setTargetPosition(-distance);
        backRight.setTargetPosition(distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(distance > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
            }
        }else {

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
    public void strafeRight(double power, double inches) {
        int distance = (int) (57.580322 * inches - 21.521799);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-distance);
        frontRight.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        backRight.setTargetPosition(-distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(distance > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {

                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
            }
        }else {
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }
    public void diagnolRight(double power, double inches) {
        int distance = (int) (57.580322 * inches - 21.521799);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-distance);
        frontRight.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        backRight.setTargetPosition(-distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(distance > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {

               // frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                //backRight.setPower(power);
            }
        }else {
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }
    public void driveSon(double power, double inches) {

        int distance = (int) (57.580322 * inches - 21.521799);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        backRight.setTargetPosition(distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //while (backLeft.getCurrentPosition() > backLeft.getTargetPosition()) {


        if(distance > 0){
            while(frontLeft.isBusy() || backLeft.isBusy()) {

                frontLeft.setPower(power);
                backLeft.setPower(power);
            }
        }
        if(distance > 0){
            while(frontRight.isBusy() || backRight.isBusy()) {

                frontRight.setPower(power);
                backRight.setPower(power);
            }
        }
        //resetAngle();
    }
    public void Correction(double x, double y)
   {

    T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
    if (up == null) return;

    double temp = up.pose.getTranslation().getY() / 0.0254-x;
    double temp1 = up.pose.getTranslation().getX() / 0.0254-y;
    double hype = (Math.pow(temp,2))+(Math.pow(temp1,2));
    double distance = Math.sqrt(hype);
    if(temp>0)
    {
        strafeLeft(.3,temp);

        //diagnolRight(.3,distance);

    }
    else if(temp1>0)
    {
        driveSon(.3,temp1);
    }
    else if (temp<0)
    {
        strafeRight(.3,temp*-1);

    }
    else if(temp1<0)
    {
        driveSon(.3,temp1*-1);
    }
    //else if(temp<0&&temp1>0)
    //{
      //  strafeRight(.3,temp*-1);
        //driveSon(.3,temp1);

    //}
    //else if (temp>0&&temp1<0)
      // {
        //   strafeRight(.3,temp);
          // driveSon(.3, temp1*-1);

       //}







}
}