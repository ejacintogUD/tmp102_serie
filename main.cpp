/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BufferedSerial.h"
#include "Semaphore.h"
#include "SerialBase.h"
#include "ThisThread.h"
#include "cmsis_os2.h"
#include "entropy.h"
#include "mbed.h"
#include <cmath>

I2C tempsensor(D14, D15); //sda, sc1
BufferedSerial pc(USBTX, USBRX); //tx, rx
UnbufferedSerial bt(PA_11, PA_12);

Thread hilo_leer_sensor(osPriorityNormal,OS_STACK_SIZE, nullptr, nullptr);
Thread hilo_enviar (osPriorityNormal,OS_STACK_SIZE, nullptr, nullptr);
Thread hilo_recibir(osPriorityNormal1,OS_STACK_SIZE, nullptr, nullptr);

Mutex bt_mutex;


Semaphore leer_enviar_smp(0);
Semaphore recibir_smp(0);


void enviar(void);
void leer_sensor(void);
void recibir(void);
void rc_bt_isr(void);


char men[30];

static int dec=0;
static int ent=0;

const int addr = 0x90;
char config_t[3];
char temp_read[2];
static float_t temp;
int main()
{
    config_t[0] = 0x01; //set pointer reg to 'config register'
    config_t[1] = 0x60; // config data byte1
    config_t[2] = 0xA0; // config data byte2

    tempsensor.write(addr, config_t, 3);
    config_t[0] = 0x00; //set pointer reg to 'data register'
    tempsensor.write(addr, config_t, 1); //send to pointer 'read temp'

    hilo_leer_sensor.start(leer_sensor);
    hilo_enviar.start(enviar);
    hilo_recibir.start(recibir);

    //bt.attach(&rc_bt_isr, SerialBase::RxIrq);
    sprintf (men, "Arranque del sistema \n\r");
    pc.write(men, 22); 

while(true) 
    {
       
       
       
    }
}


void leer_sensor (void)
{

    while(true)
    {
        tempsensor.read(addr, temp_read, 2);//read the two-byte temp data
        temp = 0.0625 * (((temp_read[0] << 8) + temp_read[1]) >> 4); //convert data
        ent = int(temp);  // parte entera 
        dec = int((temp - ent)*10000);    
        sprintf(men, "Temp = %3u,%04u degC\n\r", ent, dec);
        leer_enviar_smp.release();
        ThisThread::sleep_for(1s);
    }
}


void enviar(void)
{
    
    while(true)
    {
         leer_enviar_smp.acquire();
         bt_mutex.lock();
         //__disable_irq();
         pc.write(men, 22);
         //bt.write(men, 22);
         //__enable_irq();   

         bt_mutex.unlock();
                 
    }
    
}


void rc_bt_isr (void)
{
    recibir_smp.release();
}

void recibir(void)
{
    char c;
    while(true)
    {
        recibir_smp.acquire();
        bt_mutex.lock();
        bt.read(&c, 1);
        bt_mutex.unlock();

        if (c=='a') 
        {
        sprintf(men, "yuju\n\r");
        pc.write(men, 6);

        }
        else if (c == 'b')
        {
        sprintf(men, "F por mi\n\r");
        pc.write(men, 10);

        }  
    }
}

