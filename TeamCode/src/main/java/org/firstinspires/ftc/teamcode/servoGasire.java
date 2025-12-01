package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class servoGasire  extends LinearOpMode {
    private ServoImplEx aruncare;
    double pos= 0.5;
    @Override
    public void runOpMode(){
        aruncare = hardwareMap.get(ServoImplEx.class, "aruncare");


        waitForStart();
        while (opModeIsActive()){


            if(gamepad1.a){
                pos += 0.0001;
            }
            if(gamepad1.b){
                pos -= 0.0001;
            }


            aruncare.setPosition(pos);
            telemetry.addData("pos", pos);
            telemetry.update();
        }

    }
}
