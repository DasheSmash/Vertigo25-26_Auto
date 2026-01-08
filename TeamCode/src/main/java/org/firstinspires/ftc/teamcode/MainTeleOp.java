//Allows for this file to use other files:
package org.firstinspires.ftc.teamcode;
//Imports necessary files:
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main TeleOp", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {
    //Code that will run when the user presses 'INIT':
    @Override
    public void runOpMode() {
        //Variable declaration, initialization, and instantiation:
        ElapsedTime runtime = new ElapsedTime();
        RobotMethods RMO = new RobotMethods(hardwareMap);
        //Sets the robot's wheels to go 'backwards', and for the robot's motors to brake when at 0 power:
        RMO.SetDirectionForward();
        RMO.setZeroBehaviorAll();
        //RMO.startUsingDriveEncoders();

        //Variables to move the robot:
        double axial = 0.0;
        double lateral = 0.0;
        double yaw = 0.0;
        //Variables for drive speed, arm speed, manual arm movement, and preset arm positions:
        double driveMultiplier = 1;
        double armMultiplier = 0.75;
        double positionTo = 0.375;
        double launchPower = 0.0;
        //boolean intake = false;
        //boolean released = true;

        //Waits for the play button to be pressed:
        waitForStart();
        runtime.reset();
        //Repeatedly runs after the play button is pressed:
        while(opModeIsActive()) {
            //if(gamepad1.dpad_up){RMO.freeSlide();}
            // Pressing "back" on either of the controllers will stop the code:
            if (gamepad1.back || gamepad2.back){terminateOpModeNow();}

            //Changes where the robot will go according to the stick directions and the speed setting:
            axial = driveMultiplier * (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            lateral = driveMultiplier * (-gamepad1.left_stick_x);
            yaw = driveMultiplier * (-gamepad1.right_stick_x);
            if(Math.abs(gamepad2.left_trigger) > 0.1f){yaw += -gamepad2.right_stick_x * 0.25; lateral += gamepad2.left_stick_x * 0.25;}
            //if(Math.abs(gamepad2.left_stick_x) > 0.1f){lateral += gamepad2.left_stick_x * 0.25;}

            //Determines speed setting:
            /*if (gamepad1.right_trigger > 0.1f){driveMultiplier = 0.25;}
            else if (gamepad1.left_trigger > 0.1f){driveMultiplier = 1;}
            else{driveMultiplier = 0.5;}*/

            /*if (gamepad2.left_trigger > 0.1f){armMultiplier = 0.5;}
            else if (gamepad2.right_trigger > 0.1f){armMultiplier = 0.75;}
            else{armMultiplier = 1;}*/

            //Implements the changes to the robot's position from other parts of the code:
            RMO.move(axial,lateral,-yaw);

            if(gamepad1.a){
                //RMO.launchMotor.setPower(armMultiplier);
                launchPower = 1;
            }
            if(gamepad1.b){
                //RMO.launchMotor.setPower(-armMultiplier);
                positionTo = 0.355;
            }
            if(gamepad1.x) {
                launchPower = 0;
            }
            if(gamepad1.y){
                launchPower = 0.8;
            }
            if(gamepad1.dpad_up){
                launchPower = -0.5;
            }
            if(gamepad1.dpad_right){
                launchPower = -0.75;
            }
            if(gamepad1.dpad_down){
                launchPower = 0;
            }
            if(gamepad1.dpad_left){
                launchPower = -0.25;
            }

            //System to push balls into launcher:
            //if(gamepad2.left_bumper){RMO.loadBalls();}
            //if(gamepad2.right_bumper){RMO.resetLoader();}
            positionTo -= gamepad1.left_trigger/1000;
            positionTo += gamepad1.right_trigger/1000;
            positionTo = Math.max(Math.min(positionTo, 0.90), 0.355);
            RMO.ballLoader.setPosition(positionTo);

            if(gamepad1.left_bumper){launchPower -= 0.003;}
            if(gamepad1.right_bumper){launchPower += 0.003;}
            launchPower = Math.min(Math.max(launchPower, -1), 1);
            RMO.launchMotor.setPower(launchPower);
            //Toggle system that is currently unused:
            /*if(gamepad2.left_bumper && released){
                intake = !intake;
                released = false;
            }

            if (!gamepad2.left_bumper){released = true;}

            if(intake){
                if(gamepad2.right_bumper){
                    RMO.closeClaw();
                }else{
                    RMO.openClaw();
                }
            }*/

            /*if(gamepad1.dpad_up){RMO.startUsingDriveEncoders();}
            else if(gamepad1.dpad_down){RMO.stopUsingDriveEncoders();}*/

            //Sends data back to the driver's station:
            //telemetry.addData("Current centimeters from distance sensor: ", RMO.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("G2_RS_Y: ", gamepad2.right_stick_y);
            telemetry.addData("G2_LS_Y: ", gamepad2.left_stick_y);
            telemetry.addData("Servo pos: ", RMO.ballLoader.getPosition());
            telemetry.addData("FR pos: ", RMO.rightFrontDrive.getCurrentPosition());
            telemetry.addData("FL pos: ", RMO.leftFrontDrive.getCurrentPosition());
            telemetry.addData("BR pos: ", RMO.rightBackDrive.getCurrentPosition());
            telemetry.addData("BL pos: ", RMO.leftBackDrive.getCurrentPosition());
            telemetry.addData("Launch Power Percent: ", launchPower*100);
            telemetry.update();
        }
    }
}