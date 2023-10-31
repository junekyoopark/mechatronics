//Code by June-Kyoo Park
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <string.h>
#include <math.h> //needed for float absolute (fabs) function

#define LOOPTIME 1		// Sampling Time, reduced from 5 to 1 for optimal performance.

//RPI PIN DEFINTION//
#define MOTOR1 19   //RPi pins connected to motor driver
#define MOTOR2 26   //RPi pins connected to motor driver
#define ENCODERA 17		// Hall Sensor 
#define ENCODERB 27		// Hall Sensor B
#define PULSEPIN 18 //Pulse Pin of DAQ machine

//Pulsepin
int toggle = 0;
int toggle_before = 0;
int pulsePin;

//PID GAINS//
#define PGAIN 3000 //Proportional gain (Kp)
#define IGAIN 0  //Integral gain (Ki)
#define DGAIN 20000 // Derivative gain (Kd)

//ENCODER RELATED//
#define ENC2REDGEAR 218.7668 //Gear reduction rate compensation (Not exactly 216 in real-life)
int encA;
int encB;
int encoderPosition = 0;
float redGearPosition = 0;
float referencePosition = 0;
float errorPosition = 0;
float prevErrorPosition = 0; //USED FOR CASCADED PID, WHICH WAS NOT USED IN PRACTICE DUE TO POOR PERFORMANCE

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
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

//FOR PULSE FROM DAQ MACHINE
void funcPulsePin()
{
    pulsePin = digitalRead(PULSEPIN);
    if (pulsePin == HIGH){
        toggle++;
    }
}
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

//NUMBER OF ITERATIONS
int num;

//maximum time for ITAE stop condition, USED FOR TESTING
#define STOP_TIMER 3000 

int checkTime = 0; //used to check time when control while loop starts. Used for datalogging as well.

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

int main()
{
    //////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////
    //make Data log file
    char filename[100];
    FILE* file;
    printf("Enter the file name: ");
    scanf("%s", filename);
    file = fopen(strcat(filename,".csv"), "w+");
    //////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////
    printf("Enter the number of reference positions: ");
    scanf("%d", &num);
    //CHECK INPUT FOR ERROR
    if (num <= 0) {
        printf("Invalid number of reference positions.\n");
        return 1;
    }
    //Create refPosArray, itaeArray
    int refPosArray[num];
    float itaeArray[num];

    //Getting each reference position and putting them into refPosArray
    for (int i = 0; i < num; i++) {
        printf("Enter reference position %d: ", i+1);
        scanf("%d", &refPosArray[i]); //
    } 
    //////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////

    //RPi GPIO initialization
    wiringPiSetupGpio();
    //For pulse input
    pinMode(PULSEPIN, INPUT);		// Set PINPULSE as input
    wiringPiISR(PULSEPIN, INT_EDGE_RISING, funcPulsePin);
    //for Encoders
    pinMode(ENCODERA, INPUT);		// Set ENCODERA as input
    pinMode(ENCODERB, INPUT);		// Set ENCODERB as input
    softPwmCreate(MOTOR1, 0, 100);		// Create soft Pwm
    softPwmCreate(MOTOR2, 0, 100); 	// Create soft Pwm
    wiringPiISR(ENCODERA, INT_EDGE_BOTH, funcEncoderB);
    wiringPiISR(ENCODERB, INT_EDGE_BOTH, funcEncoderA);
    //////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////

    //wait until first pinPulse toggle.
    while(1)
    {
        if(toggle != toggle_before){
            break;
        }
        //printf("PULSE: %d", toggle);
    }
    toggle_before = toggle; //update toggle_before variable
    //////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////
    
    //iterating through i of refPosArray (iterating through reference positions)
    for(int i = 0; i < num; i++){ 
        //DATA LOGGING TIMER START//
        dataTimer = 0;

        //Define ref for this iteration
        referencePosition = refPosArray[i];

        float integral = 0;
        float prevError = 0;
        float itae = 0;

        //checking if looptime has past or not
        int checkTimeBefore = millis();
        //while ITAE time
        int whileStartTime = millis();
        //while true, break if pinPulse toggled. During testing, change while loop condition to millis()-whileStartTime < LOOPTIME
        while(1){
            checkTime = millis(); //checkTime is used to check time everytime while loop starts
            if (checkTime - checkTimeBefore > LOOPTIME){
                redGearPosition = (float)encoderPosition / ENC2REDGEAR;

                //define position error
                errorPosition = referencePosition - redGearPosition;
                //outer loop (position) PID
                integral += errorPosition*(checkTime - checkTimeBefore);
                //Calculate derivative
                float derivative = (errorPosition - prevError) / (checkTime - checkTimeBefore);

                //////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////                
                //PWM value output (PID)
                float controlOutput = (errorPosition * PGAIN) + (integral * IGAIN) + (derivative * DGAIN);
                //////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////

                //calculate ITAE, time milliseconds not seconds so that we don't have to divide by 1000 twice
                //also useful to see minute changes in ITAE during testing
                itae += ((checkTime-whileStartTime)) * fabs(errorPosition) * (checkTime - checkTimeBefore);

                printf("Reference Position: %f\n", referencePosition);
                printf("PWM: %f\n", controlOutput);
                printf("Error in Position: %f\n", errorPosition);
                printf("ITAE: %f\n", itae);

                //////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////
                //Actually give PWM output
                if (errorPosition > 0)
                {
                    softPwmWrite(MOTOR2, controlOutput);
                    softPwmWrite(MOTOR1, 0);
                }
                else
                {
                    softPwmWrite(MOTOR1, -controlOutput); //minus added since we have to go the other way
                    softPwmWrite(MOTOR2, 0);
                }
                //////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////
                
                //update prevError for position
                prevError = errorPosition;
                if (errorPosition == 0)
                { 
                    integral = 0; // prevents integral term to accumulate. if integral term is too large, system will be unstable
                }
                
                updateDataArray(); ///DATA LOGGING
                checkTimeBefore = checkTime;

            if(toggle != toggle_before){
                break;
            }
            }//if loop end

        }//while loop end
        toggle_before = toggle;    
        itaeArray[i] = itae;
    }//for loop end (refPosArray ith element)

    float totalITAE=0;
    for(int i = 0; i < num; i++) {
        totalITAE += itaeArray[i];
        //printf("%f ", itaeArray[i]);
        fprintf(file, "%.3f, ", itaeArray[i]);
    }     

    totalITAE = totalITAE/1000000.0f;

    printf("\n\n\n\n\n\n\n\nTotal ITAE: %f", totalITAE); //print total ITAE at the end

    fprintf(file, "\n");
    for (int i = 0; i < dataIndex; i++)
    {
        fprintf(file, "%.3f,%.3f\n", dataArray[i][0], dataArray[i][1]);
    }
    fclose(file); //DATA LOGGING RELATED

    return 0;
}
