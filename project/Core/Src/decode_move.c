#include "decode_move.h"
#include "encoder.h"
#include "motor.h"

float x_body;
float y_body;

float motordistance1;
float motordistance2;
float motordistance3;
float motordistance4;

static int64_t start_encoder_count1;
static int64_t start_encoder_count2;
static int64_t start_encoder_count3;
static int64_t start_encoder_count4;

void position_encoder_init()
{
    start_encoder_count1 = encoder_get_count(&encoder1);
    start_encoder_count2 = encoder_get_count(&encoder2);
    start_encoder_count3 = encoder_get_count(&encoder3);
    start_encoder_count4 = encoder_get_count(&encoder4);

    motordistance1 = 0.0f;
    motordistance2 = 0.0f;
    motordistance3 = 0.0f;
    motordistance4 = 0.0f;

    x_body = 0.0f;
    y_body = 0.0f;
}

void position_encoder()
{
    motordistance1 = (float)(encoder_get_count(&encoder1) - start_encoder_count1) 
                    * wheel_diameter * 3.1415926f / 40000.0f;
    motordistance2 = (float)(encoder_get_count(&encoder2) - start_encoder_count2) 
                    * wheel_diameter * 3.1415926f / 40000.0f;
    motordistance3 = (float)(encoder_get_count(&encoder3) - start_encoder_count3) 
                    * wheel_diameter * 3.1415926f / 40000.0f;
    motordistance4 = (float)(encoder_get_count(&encoder4) - start_encoder_count4) 
                    * wheel_diameter * 3.1415926f / 40000.0f;

    y_body = ( motordistance1 + motordistance2 + motordistance3 + motordistance4) / 4.0f;
    x_body = (-motordistance1 + motordistance2 - motordistance3 + motordistance4) / 4.0f;

    if(y_body >= 50)
    {
        MotorSetSpeed1(0);
        MotorSetSpeed2(0);
        MotorSetSpeed3(0);
        MotorSetSpeed4(0);
    }
}
