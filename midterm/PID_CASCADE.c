#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <string.h>
#include <math.h>

#define LOOPTIME 1		// Sampling Time

//RPI PIN DEFINTION//
#define MOTOR1 19   //RPi pins connected to motor driver
#define MOTOR2 26   //RPi pins connected to motor driver
#define ENCODERA 17		// Hall Sensor 
#define ENCODERB 27		// Hall Sensor B
#define PULSEPIN 18 //Pulse Pin of DAQ machine

// //Pulsepin
int toggle = 0;
int toggle_before = 0;
int pulsePin;

// PID GAINS for Velocity Control Loop
#define PGAIN_VEL 2500 // Proportional gain (Kp) for velocity control
#define IGAIN_VEL 0 // Integral gain (Ki) for velocity control
#define DGAIN_VEL 40000 // Derivative gain (Kd) for velocity control

//PID GAINS//
#define PGAIN 0.5 //Proportional gain (Kp)
#define IGAIN 0 //Integral gain (Ki)
#define DGAIN 1 // Derivative gain (Kd)

//ENCODER RELATED//
#define ENC2REDGEAR 216
int encA;
int encB;
int encoderPosition = 0;
float redGearPosition = 0;
float referencePosition = 0;
float errorPosition = 0;
float prevErrorPosition = 0; //ADDED IN DOUBLE PID

//ADDED IN DOUBLE PID
// Velocity variables
float velocity = 0;
float referenceVelocity = 0;
float errorVelocity = 0;
float prevErrorVelocity = 0;


int checkTime = 0; //used to check time when control while loop starts. Used for datalogging as well.
int checkTimeBefore = 0;

//DEFINE ENCODER VOID FUNCTIONS
void funcEncoderA()
{
    encA = digitalRead(ENCODERA);
    encB = digitalRead(ENCODERB);
    if (encA == HIGH)
    {
        if (encB == LOW) encoderPosition++;
        else encoderPosition--;
    }
    else
    {
        if (encB == LOW) encoderPosition--;
        else encoderPosition++;
    }

}

void funcEncoderB()
{
    encA = digitalRead(ENCODERA);
    encB = digitalRead(ENCODERB);
    if (encB == HIGH)
    {
        if (encA == LOW) encoderPosition--;
        else encoderPosition++;
    }
    else
    {
        if (encA == LOW) encoderPosition++;
        else encoderPosition--;
    }
}

void funcPulsePin()
{
    pulsePin = digitalRead(PULSEPIN);
    if (pulsePin == HIGH){
        toggle++;
    }
}


//NUMBER OF ITERATIONS
int num;

#define STOP_TIMER 3000 //maximum time for ITAE stop condition

// //DATALOGGING//
#define NUM_ROWS 30000
#define NUM_COLUMNS 2
#define DAQ_TIME 10000  // 10s
int dataIndex = 0;
int dataTimer = 0;
float dataArray[NUM_ROWS][NUM_COLUMNS];
void updateDataArray()
{
    dataArray[dataIndex][0] = (float)(checkTime - dataTimer) / 1000.0;
    dataArray[dataIndex][1] = redGearPosition;
    dataIndex++;
}
//////////DATA LOGGING END///////



int main()
{
    //make Data log file
    char filename[100];
    FILE* file;
    printf("Enter the file name: ");
    scanf("%s", filename);
    file = fopen(strcat(filename,".csv"), "w+");
    //


    //미지수 몇 개 --> nums (scanf로 받기)
    printf("Enter the number of reference positions: ");
    scanf("%d", &num);

    //CHECK INPUT FOR ERROR
    if (num <= 0) {
        printf("Invalid number of reference positions.\n");
        return 1;
    }
    //사이즈가 num 인 refPosArray, itaeArray 생성
    int refPosArray[num];
    float itaeArray[num];

    //array를 인덱싱하면서 (for loop), 원하는 refPosition 한 개씩 input 받기 (scanf)
    for (int i = 0; i < num; i++) {
        printf("Enter reference position %d: ", i+1);
        scanf("%d", &refPosArray[i]); //
    } //원하는 refPosition들이 담긴 array 가 하나 있는 상태


    //for pulse input
    wiringPiSetupGpio();
    // pinMode(PULSEPIN, INPUT);		// Set PINPULSE as input
    // wiringPiISR(PULSEPIN, INT_EDGE_RISING, funcPulsePin);
    pinMode(ENCODERA, INPUT);		// Set ENCODERA as input
    pinMode(ENCODERB, INPUT);		// Set ENCODERB as input

    softPwmCreate(MOTOR1, 0, 100);		// Create soft Pwm
    softPwmCreate(MOTOR2, 0, 100); 	// Create soft Pwm

    wiringPiISR(ENCODERA, INT_EDGE_BOTH, funcEncoderB);
    wiringPiISR(ENCODERB, INT_EDGE_BOTH, funcEncoderA);
    
    //wait until first pinPulse toggle
    while(1)
    {
        if(toggle != toggle_before){
            break;
        }
        //printf("PULSE: %d", toggle);
    }
    toggle_before = toggle;

    //이젠 for loop 선언할거임 (iterating through i of refPosArray)
    //for loop iterate through i 
    for(int i = 0; i < num; i++){ 

        // Initialize velocity PID loop
        float integralVelocity = 0;
        prevErrorVelocity = 0;
        //

        float integral = 0;
        float prevError = 0;
        float itae = 0;
        
        //checking if looptime has past or not
        int checkTimeBefore = millis();
        //while ITAE 시간
        int whileStartTime = millis();

        //DATA LOGGING TIMER START//
        dataTimer = 0;
        //Define ref for this iteration
        referencePosition = refPosArray[i];
        //define position error
        errorPosition = referencePosition - redGearPosition;
        //while true, break if pinPulse toggled
        while(1){
            checkTime = millis(); //checkTime is used to check time when while loop starts
            if (checkTime - checkTimeBefore > LOOPTIME){
                //////////////
                redGearPosition = (float)encoderPosition / ENC2REDGEAR;
                errorPosition = referencePosition - redGearPosition;
                // Update velocity
                velocity = (redGearPosition - prevErrorPosition) / (checkTime - checkTimeBefore);
                prevErrorPosition = redGearPosition;
                //////////////

                //outer loop (position) PID
                integral += errorPosition*(checkTime - checkTimeBefore);
                //Calculate derivative
                float derivative = (errorPosition - prevError) / (checkTime - checkTimeBefore);

                //PWM value output (PID)
                float controlOutput = (errorPosition * PGAIN) + (integral * IGAIN) + (derivative * DGAIN);


                referenceVelocity = controlOutput; //the referenceVelocity is the result from the position PID controlOutput

                errorVelocity = referenceVelocity - velocity;
                //inner loop (velocity) PID
                integralVelocity += errorVelocity*(checkTime - checkTimeBefore);
                float derivativeVelocity = (errorVelocity - prevErrorVelocity) / (checkTime - checkTimeBefore);
                float controlOutputVelocity = (errorVelocity * PGAIN_VEL) + (integralVelocity * IGAIN_VEL) + (derivativeVelocity * DGAIN_VEL);


                //calculate ITAE
                itae += ((millis()-whileStartTime)) * fabs(errorPosition) * ((checkTime - checkTimeBefore));

                printf("Reference Position: %f\n", referencePosition);
                printf("PWM: %f\n", controlOutputVelocity);
                printf("Error in Position: %f\n", errorPosition);
                printf("ITAE: %f\n", itae);

                //이제 구동
                if (errorPosition > 0)
                {
                    softPwmWrite(MOTOR2, controlOutputVelocity);
                    softPwmWrite(MOTOR1, 0);
                }
                else
                {
                    softPwmWrite(MOTOR1, -controlOutputVelocity); //minus added since we have to go the other way
                    softPwmWrite(MOTOR2, 0);
                }
                
                //update prevErrorVelocity
                prevErrorVelocity = errorVelocity;
                //update prevError for position
                prevError = errorPosition;

                if (errorPosition == 0)
                { 
                    integralVelocity = 0;
                    integral = 0; // prevents integral term to accumulate. if integral term is too large, system will be unstable
                }
                


                
                updateDataArray(); ///DATA LOGGING
                checkTimeBefore = checkTime;

            if(toggle != toggle_before){
                break;
            }
            }//if loop end

        }//while loop end
        // toggle_before = toggle;    
        itaeArray[i] = itae;
    }//for loop end (refPosArray ith element)

    float totalITAE=0;
    for(int i = 0; i < num; i++) {
        totalITAE += itaeArray[i];
        printf("%f ", itaeArray[i]);
        fprintf(file, "%.3f, ", itaeArray[i]);
    }     

    totalITAE = totalITAE/1000000.0f;

    printf("\n\n\n\n\n\n\n\nTotal ITAE: %f", totalITAE);
    fprintf(file, "\n");
    for (int i = 0; i < dataIndex; i++)
    {
        fprintf(file, "%.3f,%.3f\n", dataArray[i][0], dataArray[i][1]);
    }
    fclose(file); //DATA LOGGING RELATED

    return 0;
}
