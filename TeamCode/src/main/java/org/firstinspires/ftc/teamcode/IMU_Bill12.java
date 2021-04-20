  package org.firstinspires.ftc.teamcode;

 import com.acmerobotics.dashboard.FtcDashboard;
 import com.acmerobotics.dashboard.canvas.Canvas;
 import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
 import com.arcrobotics.ftclib.geometry.Pose2d;
 import com.arcrobotics.ftclib.geometry.Rotation2d;
 import com.arcrobotics.ftclib.geometry.Transform2d;
 import com.arcrobotics.ftclib.geometry.Translation2d;
 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.spartronics4915.lib.T265Camera;

 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


 @Autonomous
 public class IMU_Bill12 extends LinearOpMode {
     private static T265Camera slamra = null;

     private final FtcDashboard dashboard = FtcDashboard.getInstance();

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    double error = 10;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    @Override
    public void runOpMode() throws InterruptedException {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

       frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        if(up.confidence.equals(true))
        {
            slamra.setPose(new Pose2d(-0 * 0.0254, 0 * 0.0254, new Rotation2d(0)));
        }

        slamra.setPose(new Pose2d(-0 * 0.0254, 0 * 0.0254, new Rotation2d(0)));
        slamra.start();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(200);
            idle();
        }

        sleep(500);

        telemetry.addData("Mode", "waiting for start");
        telemetry.speak("Yash is god lol");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("X:",up.pose.getTranslation().getX() / 0.0254);
        telemetry.addData("Y:",up.pose.getTranslation().getY() / 0.0254);
        telemetry.addData("Pheta:",up.pose.getRotation().getDegrees());
        telemetry.addData("Confidence",up.confidence);

        telemetry.update();
        //telemetry.update();

        waitForStart();

        correction = checkDirection();


        driveForwardIMU(0.4,5);
        while(opModeIsActive())
        {
            telemetry.addData("X:",up.pose.getTranslation().getX() / 0.0254);
            telemetry.addData("Y:",up.pose.getTranslation().getY() / 0.0254);
            telemetry.addData("Pheta:",up.pose.getRotation().getDegrees());
            telemetry.update();

            if(gamepad1.dpad_down){
                //Correction2(0,0,0);
                Correction(0,0);
                loop1();

            }




        }
        if(!opModeIsActive())
        {
            slamra.stop();
        }







 }

    private void resetAngle() {
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;



        Rotation2d pog = up.pose.getRotation();


        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public void rotate(double power, int degrees) {
        resetAngle();

        double leftPower, rightPower;


        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(-leftPower);
        frontRight.setPower(-rightPower);
        backLeft.setPower(-leftPower);
        backRight.setPower(-rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                correction = checkDirection();

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                correction = checkDirection();

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                correction = checkDirection();

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
            }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        sleep(100);

        resetAngle();


    }
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
       double correction, angle, gain = .1;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void driveBackwardIMU(double power, double inches) {
        resetAngle();

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

        if (distance > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                frontLeft.setPower(power + correction);
                frontRight.setPower(power - correction);
                backLeft.setPower(power + correction);
                backRight.setPower(power - correction);
                correction = checkDirection();
                telemetry.addData("distance = ", distance);

                telemetry.addData("front left = ", frontLeft.getCurrentPosition());
                telemetry.addData("front right = ", frontRight.getCurrentPosition());
                telemetry.addData("back left = ", backLeft.getCurrentPosition());
                telemetry.addData("back right = ", backRight.getCurrentPosition());
                telemetry.update();

            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();

    }
    public void strafeRightIMU(double power, double inches) {
        resetAngle();

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

        if (distance > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
               frontLeft.setPower(power - correction);
               frontRight.setPower(power - correction);
               backLeft.setPower(power - correction);
               backRight.setPower(power - correction);

                correction = checkDirection();
                telemetry.addData("distance = ", distance);

                telemetry.addData("front left = ", frontLeft.getCurrentPosition());
                telemetry.addData("front right = ", frontRight.getCurrentPosition());
                telemetry.addData("back left = ", backLeft.getCurrentPosition());
                telemetry.addData("back right = ", backRight.getCurrentPosition());
                telemetry.update();

            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();

    }
    public void splineBR(double power, int inches) {
        resetAngle();

        int distance = (int) (57.580322 * inches - 21.521799);

         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         frontRight.setTargetPosition(distance);
         backLeft.setTargetPosition(distance);

         frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (distance > 0) {
            while (frontRight.isBusy() && backLeft.isBusy()) {
                frontRight.setPower(power - correction);
                backLeft.setPower(power + correction);
                correction = checkDirection();

                telemetry.addData("distance = ", distance);
                telemetry.addData("front right = ", frontRight.getCurrentPosition());
                telemetry.addData("back left = ", backLeft.getCurrentPosition());
                telemetry.update();

            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void splineFL(double power, int inches) {
        resetAngle();

        int distance = (int) (57.580322 * inches - 21.521799);

         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         frontRight.setTargetPosition(-distance);
         backLeft.setTargetPosition(-distance);

         frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (distance > 0) {
            while (frontRight.isBusy() && backLeft.isBusy()) {
                frontRight.setPower(power - correction);
                backLeft.setPower(power + correction);
                correction = checkDirection();

                telemetry.addData("distance = ", distance);
                telemetry.addData("front right = ", frontRight.getCurrentPosition());
                telemetry.addData("back left = ", backLeft.getCurrentPosition());
                telemetry.update();

            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void splineBL(double power, int inches) {
        resetAngle();

        int distance = (int) (57.580322 * inches - 21.521799);

         backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         frontLeft.setTargetPosition(distance);
         backRight.setTargetPosition(distance);

         frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (distance > 0) {
            while (frontLeft.isBusy() &&  backRight.isBusy()) {
                frontLeft.setPower(power - correction);
                backRight.setPower(power + correction);
                correction = checkDirection();

                telemetry.addData("distance = ", distance);
                telemetry.addData("front left = ", frontLeft.getCurrentPosition());
                telemetry.addData("back right = ", backRight.getCurrentPosition());
                telemetry.update();

            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void splineFR(double power, int inches) {
        resetAngle();

        int distance = (int) (57.580322 * inches - 21.521799);

         backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         frontLeft.setTargetPosition(-distance);
         backRight.setTargetPosition(-distance);

         frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (distance > 0) {
            while (frontLeft.isBusy() &&  backRight.isBusy()) {
                frontLeft.setPower(power - correction);
                backRight.setPower(power + correction);
                correction = checkDirection();

                telemetry.addData("distance = ", distance);
                telemetry.addData("front left = ", frontLeft.getCurrentPosition());
                telemetry.addData("back right = ", backRight.getCurrentPosition());
                telemetry.update();

            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void driveSon(double power, int inches) {

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
        resetAngle();
    }
    public void driveForwardIMU(double power, double inches) {
        resetAngle();

        int distance = -(int) (57.580322 * inches - 21.521799);

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

        if (distance < 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                frontLeft.setPower(-power + correction);
                frontRight.setPower(-power - correction);
                backLeft.setPower(-power + correction);
                backRight.setPower(-power - correction);
                correction = checkDirection();
                telemetry.addData("distance = ", distance);

                telemetry.addData("front left = ", frontLeft.getCurrentPosition());
                telemetry.addData("front right = ", frontRight.getCurrentPosition());
                telemetry.addData("back left = ", backLeft.getCurrentPosition());
                telemetry.addData("back right = ", backRight.getCurrentPosition());
                telemetry.update();

            }
        }
        // turn the motors off.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();


    }


    public void strafeRight(double power, int inches) {
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
    public void spinLeft(double power, int inches) {

        int distance = (int) (57.580322 * inches - 21.521799);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(-distance);
        backLeft.setTargetPosition(distance);
        backRight.setTargetPosition(-distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(distance > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);
            }
        }
    }
    public void spinRight(double power, int inches) {

        int distance = (int) (57.580322 * inches - 21.521799);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-distance); //forward
        frontRight.setTargetPosition(distance);
        backLeft.setTargetPosition(-distance);
        backRight.setTargetPosition(distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(distance > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);
            }
        }

    }
     public void splineFL2(double power, int inches) {
         int ticks = (int) (inches*383.6/(50/25.4*2*Math.PI));

         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         frontRight.setTargetPosition(ticks);
         backLeft.setTargetPosition(ticks);
         frontLeft.setTargetPosition(-ticks/2);
         backRight.setTargetPosition(-ticks/2);

         frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         if (ticks > 16) {
             while (frontRight.isBusy() && backLeft.isBusy()) {

                 frontRight.setPower(power - correction);
                 backLeft.setPower(power + correction);
                 frontLeft.setPower((power*0.5) + correction);
                 backRight.setPower((power*0.5) - correction);
                 correction = checkDirection();

             }
         }
         // reset angle tracking on new heading.
         resetAngle();


     }
    public void stop(double power, int inches)
    {
        int distance = (int) (57.580322 * inches - 21.521799);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
     public void splineFR2(double power, int inches) {
         int ticks = (int) (inches*383.6/(50/25.4*2*Math.PI));

         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         frontLeft.setTargetPosition(ticks);
         backRight.setTargetPosition(ticks);
         backLeft.setTargetPosition(-ticks/2);
         frontRight.setTargetPosition(-ticks/2);

         frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         if (ticks > 16) {
             while (frontLeft.isBusy() && backRight.isBusy()) {

                 frontLeft.setPower(power - correction);
                 backRight.setPower(power + correction);
                 frontRight.setPower((power*0.5) + correction);
                 backLeft.setPower((power*0.5) - correction);
                 correction = checkDirection();

             }
         }
         // reset angle tracking on new heading.
         resetAngle();


     }
     public void splineBR2(double power, int inches) {
         int ticks = (int) (inches*383.6/(50/25.4*2*Math.PI));

         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         backLeft.setTargetPosition(ticks);
         frontRight.setTargetPosition(ticks);
         backRight.setTargetPosition(-ticks/2);
         frontLeft.setTargetPosition(-ticks/2);

         frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         if (ticks > 16) {
             while (backLeft.isBusy() && frontRight.isBusy()) {

                 frontRight.setPower(power - correction);
                 backLeft.setPower(power + correction);
                 frontLeft.setPower((power*0.5) + correction);
                 backRight.setPower((power*0.5) - correction);
                 correction = checkDirection();

             }
         }
         // reset angle tracking on new heading.
         resetAngle();


     }



     public void splineBL2(double power, int inches) {
         int ticks = (int) (inches*383.6/(50/25.4*2*Math.PI));

         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         frontLeft.setTargetPosition(-ticks);
         backRight.setTargetPosition(-ticks);
         backLeft.setTargetPosition(ticks/2);
         frontRight.setTargetPosition(ticks/2);

         frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         if (ticks > 16) {
             while (frontLeft.isBusy() && backRight.isBusy()) {

                 frontLeft.setPower(power - correction);
                 backRight.setPower(power + correction);
                 frontRight.setPower((power*0.5) + correction);
                 backLeft.setPower((power*0.5) - correction);
                 correction = checkDirection();

             }
         }
         // reset angle tracking on new heading.
         resetAngle();


     }



     public void prop(double power, int ticks)
     {
         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         double inputt = frontLeft.getCurrentPosition();
         double error = ticks - inputt;



     }
     public void Correction(double x, double y)
     {

         T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
         if (up == null) return;

         double temp = up.pose.getTranslation().getY() / 0.0254-x;
         double temp1 = up.pose.getTranslation().getX() / 0.0254-y;
         double hype = (Math.pow(temp,2))+(Math.pow(temp1,2));
         double distance = Math.sqrt(hype);
         Rotation2d pog = up.pose.getRotation();
         int potato = (int) pog.getDegrees();
         if(temp>0)
         {
             strafeLeft(.3,temp);

             //diagnolRight(.3,distance);

         }
           if(temp1>0)
         {
             driveBackwardIMU(.6,temp1);
         }
           if (temp<0)//add else if stuff if we wanna adjsut correction
         {
             strafeRightIMU(.6,temp*-1);

         }
           if(temp1<0)
         {
             driveForwardIMU(.6,temp1*-1);
         }
           if (potato>4&&potato<90)
          {
              rotate(.2,potato*-1);
          }
         if (potato<-4&&potato>-90)//pheta goes to negative degrees, add a buffer.
         {
             rotate(.2,potato*-1);
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
     public void Correction3(double x, double y)
     {

         T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
         if (up == null) return;

         double temp = up.pose.getTranslation().getY() / 0.0254-x;
         double temp1 = up.pose.getTranslation().getX() / 0.0254-y;
         double hype = (Math.pow(temp,2))+(Math.pow(temp1,2));
         double distance = Math.sqrt(hype);
         Rotation2d pog = up.pose.getRotation();
         int potato = (int) pog.getDegrees();
         if(temp>0)
         {
             strafeLeft(.3,temp);

             //diagnolRight(.3,distance);

         }
         if(temp1>0)
         {
             driveBackwardIMU(.6,temp1);
         }
         if (temp<0)//add else if stuff if we wanna adjsut correction
         {
             strafeRightIMU(.6,temp*-1);

         }
         if(temp1<0)
         {
             driveForwardIMU(.6,temp1*-1);
         }
         if (potato>4&&potato<90)
         {
             rotate(.2,potato*-1);
         }
         if (potato<-4&&potato>-90)//pheta goes to negative degrees, add a buffer.
         {
             rotate(.2,potato*-1);
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
     public void Correction2(double x, double y,double pheta)
     {

         T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
         if (up == null) return;
         //
         int distance = 0;
         //Get the distance we need to move based on x and Y
         double X = (int) (up.pose.getTranslation().getX() / 0.0254)-(x);
         double Y = (int) (up.pose.getTranslation().getY() / 0.0254)-(y);
         //rotate caclculations
         Rotation2d getRotate = up.pose.getRotation();
         int Rotation = (int) getRotate.getDegrees();
         int RotateCorrect= (int) (Rotation-pheta);
         //math for Diagonal travel
         double hype = (int)(Math.pow(X,2))+(Math.pow(Y,2));
         if(hype>0)
         {
             distance = (int) (Math.sqrt(hype));
         }
         //rotate travel
         //while (X!=0||Y!=0||RotateCorrect!=0) {
//             if (RotateCorrect > 3)//should i add a while loop??????
//             {
//                 rotate(.3, RotateCorrect * -1);
//             }
//             if (RotateCorrect < -3) {
//                 rotate(.3, RotateCorrect);
//             }
         //Diagonal travel
         if (distance != 0) {
             if (X > 0 && Y < 0) {
                 splineFR2(.3, distance);//this doesnt work Switched to FL2(back need to be tested) maybe swithc these two
                 telemetry.addData("SPLINEBR = MATH ", X);
                 telemetry.addData("SpllineBR = MATH", Y);
                 telemetry.addData("POSx ", up.pose.getTranslation().getY() / 0.0254);
                 telemetry.addData("POSy", up.pose.getTranslation().getX() / 0.0254);
                 telemetry.update();
             }
             if (X < 0 && Y < 0) {//dont touch condition
                 splineBL2(.3, distance);//this only works
                 telemetry.addData("FR = MATH ", X);
                 telemetry.addData("FR = MATH", Y);
                 telemetry.addData("POSx ", up.pose.getTranslation().getY() / 0.0254);
                 telemetry.addData("POSy", up.pose.getTranslation().getX() / 0.0254);
                 telemetry.update();
             }
             if (X > 0 && Y > 0) {//dont touch condition
                 splineFL2(.3, distance);//this also works(back need to be tested)//maybe it doesnt work
                 telemetry.addData("SPLINEBL = MATH ", X);
                 telemetry.addData("SpllineBL = MATH", Y);
                 telemetry.addData("POSx ", up.pose.getTranslation().getY() / 0.0254);
                 telemetry.addData("POSy", up.pose.getTranslation().getX() / 0.0254);
                 telemetry.update();
             }
             if (X < 0 && Y > 0) {
                 splineBR2(.3, distance);//does this work, switched to BR
                 telemetry.addData("SPLINEFL = MATH ", X);
                 telemetry.addData("SpllineFL = MATH", Y);
                 telemetry.addData("POSx ", up.pose.getTranslation().getY() / 0.0254);
                 telemetry.addData("POSy", up.pose.getTranslation().getX() / 0.0254);
                 telemetry.update();
             }//if pi = 3.14 does pie also = 3.14

         }
//             if (RotateCorrect > 0)

//             {
//                 rotate(.3, RotateCorrect * -1);
//             }
//             if (RotateCorrect < 0) {
//                 rotate(.3, RotateCorrect);
//             }
         //no Diagonal travel
         else {
             if (X < 0) {
                 strafeLeft(.3, X * -1);
             }
             if (X > 0) {
                 strafeRightIMU(.3, X);
             }
             if (Y > 0) {
                 driveForwardIMU(.3, Y);
             }
             if (Y < 0) {
                 driveBackwardIMU(.3, Y * -1);
             }

             //     }
         }

     }
     public void loop1()
     {
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
     }

 }




