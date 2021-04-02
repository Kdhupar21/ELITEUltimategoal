
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous

public class WebcamDetect extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    double ftpower = 0.6;
    int rings = -1;

    private DcMotor frontLeft,
            frontRight,
            backLeft,
            backRight,
            intake,
            middleWheel,
            wobble,
            shooter;

    private Servo left, front, cam;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ASam/S7/////AAABmVxoib78w0fCiyGeawjRMf4QUyzHQ8/jjeq54mP3tIf5oMlJ+N4QHM00febVooNfM4B+yBPzIg6u5Y5d76SumAEy+lIEL7oSQN0JguZhNpOIgmmOC6uwwJE/TFyMprruEcDNWcK+cUh8vKV0ApX5LxLrAd4K+6rghMDk6OtUdvs4m3bwR16JHAlBFHNfmYZfeyV9MVEIsVBu0Td17OHfo0hBj5QeBtD54FOhk1IMmyz1agBEPnzo9R8Uk3RKmUn993R4f04q6uBYY90KNmnAIFGhBDmHC+tS0vlTeAhxSbK3UfpIdRgPZcIxJhH1DCDzrGdxw6xo9UuH4qxF26yUhAURRdLcOMhgd7cJBjvJcZN3";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    double error = 10;

    //   BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(DcMotor.class, "I");
        middleWheel = hardwareMap.get(DcMotor.class, "MW");
        wobble = hardwareMap.get(DcMotor.class, "W");
        shooter = hardwareMap.get(DcMotor.class, "S");
        left = hardwareMap.get(Servo.class, "wobble");
        front = hardwareMap.get(Servo.class, "front");
        cam = hardwareMap.get(Servo.class, "Cam");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        middleWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

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
        telemetry.update();

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        //start of auto
        waitForStart();

        front.setPosition(0.7);
        driveForwardIMU(0.2,1000);
        sleep(900);





        // rotate(77,0.3);
        // sleep(1000);
        // rotate(-77,0.3);


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            if(recognition.getLabel() == "Quad")
                            {
                                rings = 4;
                            }
                            else if(recognition.getLabel() == "Single")
                            {
                                rings = 1;
                            }
                            // else if(recognition.getLabel() != "Single" && recognition.getLabel() != "Quad" )
                            // {
                            //     rings = 0;
                            // }

                        }
                    }
                    // else if(recognition.getLabel() != "Single" && recognition.getLabel() != "Quad" )
                    // {
                    //       rings = 0;
                    // }
                    telemetry.addData("Ring Count", rings);
                    telemetry.update();
                    if(rings==4)
                    {
                        front.setPosition(0.7);
                        sleep(700);
                        driveForwardIMU(0.3,2250);
                        sleep(1000);
                        splineFR(0.4,468);
                        sleep(1000);
                        front.setPosition(0.4);
                        sleep(1350);
                        splineBL(0.4,1200);
                        sleep(1000);
                        strafeLeftIMU(0.3,500);
                        driveBackwardIMU(0.2,1000);
                        shoot(0.5,4000);
                        driveForwardIMU(0.4,500);
                        sleep(20000);


                    }
                    else if (rings==1)
                    {
                        front.setPosition(0.8);
                        sleep(600);
                        driveForwardIMU(0.4,1700);
                        sleep(600);
                        strafeRightIMU(0.3,500);
                        sleep(600);
                        front.setPosition(0.4);
                        sleep(600);
                        strafeRightIMU(0.3,200);
                        driveBackwardIMU(0.3,1000);
                        sleep(600);
                        shoot(0.5,4000);
                        down(0.3,700);
                        left.setPosition(0.1);
                        driveBackwardIMU(0.2,750);
                        driveBackwardIMU(0.1,200);
                        sleep(500);
                        left.setPosition(0.7);
                        sleep(500);
                        driveForwardIMU(0.4,1700);
                        sleep(300);
                        spin(0.3,1400);
                        left.setPosition(0.1);
                        sleep(600);
                        spin(0.3,100);
                        driveForwardIMU(0.6,200);
                        sleep(20000);

                        //strafeRightIMU(0.3,7);

                    }
                    else if (rings == -1)
                    {
                        front.setPosition(0.7);
                        sleep(700);
                        driveForwardIMU(0.4,800);
                        sleep(1000);
                        splineFR(0.3,400);
                        sleep(800);
                        front.setPosition(0.4);
                        sleep(1300);
                        splineBL(0.3,350);
                        sleep(800);
                        strafeRightIMU(0.3,595);
                        //driveForwardIMU(0.4,400);
                        //sleep(500);
                        shoot(0.48,4000);
                        down(0.3,700);
                        left.setPosition(0.1);
                        driveBackwardIMU(0.2,800);
                        driveBackwardIMU(0.1,200);
                        sleep(500);
                        left.setPosition(0.7);
                        sleep(500);
                        driveForwardIMU(0.4,1700);
                        sleep(300);
                        spin(0.3,730);
                        driveBackwardIMU(0.4,425);
                        left.setPosition(0.1);
                        sleep(600);
                        spin(0.3,100);
                        driveForwardIMU(0.6,350);
                        sleep(20000);


                    }
                }
            }


        }


        if (tfod != null) {
            tfod.shutdown();
        }

    }
    private void resetAngle() {
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
    public void shoot(double power, int ticks) {

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setTargetPosition(ticks);
        middleWheel.setTargetPosition(ticks);

        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(ticks > 0)
        {
            while (shooter.isBusy())
            {
                shooter.setPower(power);
                middleWheel.setPower(power);
            }
        }

        shooter.setPower(0);
        middleWheel.setPower(0);

    }
    public void lift(double power, int ticks) {

        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobble.setTargetPosition(ticks);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(ticks > 0)
        {
            while (wobble.isBusy())
            {
                wobble.setPower(power);
            }
        }

        wobble.setPower(0);

    }
    public void down(double power, int ticks) {

        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobble.setTargetPosition(-ticks);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(ticks > 0)
        {
            while (wobble.isBusy())
            {
                wobble.setPower(power);
            }
        }

        wobble.setPower(0);

    }
    public void sweep(double power, int ticks)
    {
        middleWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleWheel.setTargetPosition(ticks);
        middleWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleWheel.setPower(power);


        while (middleWheel.getCurrentPosition() < middleWheel.getTargetPosition())
        {
            //lool
        }

        middleWheel.setPower(0);
        middleWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void load()
    {
        double power = 0.3;
        int ticks = 9000;

        middleWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleWheel.setTargetPosition(ticks);
        middleWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleWheel.setPower(power);

        while (middleWheel.isBusy())
        {

        }

        middleWheel.setPower(0);
        middleWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        frontRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .01;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void driveForwardIMU(double power, int ticks) {

        int errorFL = ticks - frontLeft.getCurrentPosition();
        int errorFR = ticks - frontRight.getCurrentPosition();
        int errorBL = ticks - backLeft.getCurrentPosition();
        int errorBR = ticks - backRight.getCurrentPosition();
        resetAngle();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (ticks > 16) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy())
            {

                frontLeft.setPower(power - correction );
                frontRight.setPower(power + correction );
                backLeft.setPower(power - correction );
                backRight.setPower(power + correction );
                correction = checkDirection();

                telemetry.addData("goal = ", ticks);

                telemetry.addData("front left = ", frontLeft.getCurrentPosition());
                telemetry.addData("front right = ", frontRight.getCurrentPosition());
                telemetry.addData("back left = ", backLeft.getCurrentPosition());
                telemetry.addData("back right = ", backRight.getCurrentPosition());
                telemetry.update();

            }
        }
        // reset angle tracking on new heading.
        resetAngle();

    }
    public void spin(double power, int ticks) {

        int errorFL = ticks - frontLeft.getCurrentPosition();
        int errorFR = ticks - frontRight.getCurrentPosition();
        int errorBL = ticks - backLeft.getCurrentPosition();
        int errorBR = ticks - backRight.getCurrentPosition();
        resetAngle();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (ticks > 16) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy())
            {

                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
                //correction = checkDirection();

                telemetry.addData("goal = ", ticks);

                telemetry.addData("front left = ", frontLeft.getCurrentPosition());
                telemetry.addData("front right = ", frontRight.getCurrentPosition());
                telemetry.addData("back left = ", backLeft.getCurrentPosition());
                telemetry.addData("back right = ", backRight.getCurrentPosition());
                telemetry.update();

            }
        }
        // reset angle tracking on new heading.
        resetAngle();

    }
    public void driveBackwardIMU(double power, int ticks) {

        int errorFL = ticks - frontLeft.getCurrentPosition();
        int errorFR = ticks - frontRight.getCurrentPosition();
        int errorBL = ticks - backLeft.getCurrentPosition();
        int errorBR = ticks - backRight.getCurrentPosition();
        resetAngle();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (ticks > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                //   frontLeft.setPower(power + correction);
                //   frontRight.setPower(power - correction);
                //   backLeft.setPower(power + correction);
                //   backRight.setPower(power - correction);
                //   correction = checkDirection();

                frontLeft.setPower(power + correction );
                frontRight.setPower(power + correction );
                backLeft.setPower(power + correction );
                backRight.setPower(power + correction );
                correction = checkDirection();

                telemetry.addData("goal = ", ticks);

                telemetry.addData("front left = ", frontLeft.getCurrentPosition());
                telemetry.addData("front right = ", frontRight.getCurrentPosition());
                telemetry.addData("back left = ", backLeft.getCurrentPosition());
                telemetry.addData("back right = ", backRight.getCurrentPosition());
                telemetry.update();

            }
        }
        // reset angle tracking on new heading.
        resetAngle();

    }
    public void strafeRightIMU(double power, int ticks) {
        resetAngle();

        // int distance = (int) (57.580322 * inches - 21.521799);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (ticks > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                frontLeft.setPower(power - correction);
                frontRight.setPower(power - correction);
                backLeft.setPower(power - correction);
                backRight.setPower(power - correction);

                correction = checkDirection();
                telemetry.addData("distance = ", ticks);

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
    public void strafeLeftIMU(double power, int ticks) {
        resetAngle();

        //  int distance = (int) (57.580322 * inches - 21.521799);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (ticks > 0) {
            while (frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                frontLeft.setPower(power - correction);
                frontRight.setPower(power - correction);
                backLeft.setPower(power - correction);
                backRight.setPower(power - correction);

                correction = checkDirection();
                telemetry.addData("distance = ", ticks);

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
    public void splineFL(double power, int ticks) {

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
    public void splineFR(double power, int ticks) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks/2);
        backLeft.setTargetPosition(-ticks/2);

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
    public void splineBL(double power, int ticks) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks/2);
        backLeft.setTargetPosition(ticks/2);

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
    public void splineBR(double power, int ticks) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        frontLeft.setTargetPosition(ticks/2);
        backRight.setTargetPosition(ticks/2);

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


    public void strafeLeft(double power, int inches) {
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


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}


