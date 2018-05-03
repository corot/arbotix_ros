/* 
 ArbotiX Firmware for ROS driver
 Copyright (c) 2008-2012 Vanadium Labs LLC.  All right reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of Vanadium Labs LLC nor the names of its 
 contributors may be used to endorse or promote products derived 
 from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros.h"
#include <CM9_BC.h>

#define AX_READ_DATA                2
#define AX_WRITE_DATA               3

#define ERR_CHECKSUM       16

#define CONTROLLER_COUNT    5

/* Hardware Constructs */
Dynamixel Dxl(1);
Dynamixel *pDxl = &Dxl;
BioloidController bioloid;

/* Register Storage */
unsigned char baud = 7;         // ?
unsigned char ret_level = 1;    // ?
unsigned char alarm_led = 0;    // ?

/* Pose & Sequence Structures */
typedef struct
{
  bc_pose_t pose;                // index of pose to transition to 
  int time;                      // time for transition
} sp_trans_t;
int poses[30][CONTROLLER_COUNT]; // poses [index][servo_id-1]
sp_trans_t sequence[50];         // sequence
int seqPos;                      // step in current sequence

void blink(unsigned int times = 1)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(BOARD_LED_PIN, HIGH); // set to as HIGH LED is turn-off
    delay(100);          // Wait for 0.1 second
    digitalWrite(BOARD_LED_PIN, LOW);  // set to as LOW LED is turn-on
    delay(100);          // Wait for 0.1 second
  }
}

void setup()
{
  Dxl.begin(1);
  SerialUSB.begin();

  bioloid.setup(CONTROLLER_COUNT);
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, LOW);

  blink(3);
//  userSetup();
//  pinMode(0,OUTPUT);     // status LED
}

int doPlaySeq()
{
  seqPos = 0;
  int i;
  while (sequence[seqPos].pose != 0xff)
  {
    // are we HALT?
    if (SerialUSB.available() && SerialUSB.read() == 'H')
      return 1;

    int p = sequence[seqPos].pose;
    int poseSize = bioloid.getPoseSize();

    // load pose
    for (i = 0; i < poseSize; i++)
    {
      bioloid.setNextPose(i + 1, poses[p][i]);
    }
    // interpolate
    bioloid.interpolateSetup(sequence[seqPos].time);
    while (bioloid.interpolating())
    {
      bioloid.interpolateStep();
    }
    // next transition
    seqPos++;
  }
  return 0;
}

/*
 * Send status packet
 */
void statusPacket(int id, int err)
{
  SerialUSB.write(0xff);
  SerialUSB.write(0xff);
  SerialUSB.write(id);
  SerialUSB.write(2);
  SerialUSB.write(err);
  SerialUSB.write(255 - ((id + 2 + err) % 256));
}

/* 
 * decode packets: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix 
 */
void loop()
{
  int i, poseSize;
  
  // process messages
  while (SerialUSB.available() > 0)
  {
    // We need two 0xFF at start of packet
    if (mode == 0)         // start of new packet
    {
      if (SerialUSB.read() == 0xff)
      {
        mode = 2;
//        digitalWrite(0, HIGH - digitalRead(0));
      }
      //}else if(mode == 1){   // another start byte
      //    if(SerialUSB.read() == 0xff)
      //        mode = 2;
      //    else
      //        mode = 0;
    }
    else if (mode == 2)   // next byte is index of servo
    {
      id = SerialUSB.read();
      if (id != 0xff)
        mode = 3;
    }
    else if (mode == 3)   // next byte is length
    {
      length = SerialUSB.read();
      checksum = id + length;
      mode = 4;
    }
    else if (mode == 4)   // next byte is instruction
    {
      ins = SerialUSB.read();
      checksum += ins;
      index = 0;
      mode = 5;
    }
    else if (mode == 5)   // read data in
    {
      params[index] = SerialUSB.read();
      checksum += (int) params[index];
      index++;
      if (index + 1 == length)  // we've read params & checksum
      {
        mode = 0;
        if ((checksum % 256) != 255)
        {
          // return an error packet: FF FF id Len Err=bad checksum, params=None check
          statusPacket(id, ERR_CHECKSUM);
        }
        else if (id == 253)
        {  // ID = 253, ArbotiX instruction
          switch (ins)
          {
          case AX_WRITE_DATA:
            // send return packet
  blink(2);
  //            statusPacket(id, handleWrite());
            break;

          case AX_READ_DATA:
  blink(4);
  //    LED blinks when an unanswererd ins is received  TODO everywhere
//            checksum = id + params[1] + 2;
//            SerialUSB.write(0xff);
//            SerialUSB.write(0xff);
//            SerialUSB.write(id);
//            SerialUSB.write((unsigned char) 2 + params[1]);
//            SerialUSB.write((unsigned char) 0);
//            // send actual data
//            checksum += handleRead();
//            SerialUSB.write(255 - ((checksum) % 256));
            break;

          case ARB_SIZE_POSE: // Pose Size = 7, followed by single param: size of pose
            statusPacket(id, 0);
            bioloid.setPoseSize(params[0]);
            bioloid.readPose();
            break;

          case ARB_LOAD_POSE: // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size)
            statusPacket(id, 0);

            poseSize = bioloid.getPoseSize();
            for (i = 0; i < poseSize; i++)
            {
              poses[params[0]][i] = (params[(2 * i) + 1]
                  + (params[(2 * i) + 2] << 8));
            }
            break;

          case ARB_LOAD_SEQ: // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size) 
            statusPacket(id, 0);
            for (i = 0; i < (length - 2) / 3; i++)
            {
              sequence[i].pose = params[(i * 3)];
              sequence[i].time = params[(i * 3) + 1]
                  + (params[(i * 3) + 2] << 8);
            }
            break;

          case ARB_PLAY_SEQ:                   // Play Seq = A, no params   
            statusPacket(id, 0);
            doPlaySeq();
            break;

          case ARB_LOOP_SEQ:               // Play Seq until we recieve a 'H'alt
            statusPacket(id, 0);
            while (doPlaySeq() > 0)
              ;
            break;

            // ARB_TEST is deprecated and removed
          case ARB_CONTROL_SETUP:              // Setup a controller
            statusPacket(id, 0);
//            if (params[0] < CONTROLLER_COUNT)
      //      {
  //            //TODO how to set servo ID??? setId  But what are the other values???  maybe setResolution?
    //          bioloid.setId(params[0], params[1]
              //controllers[params[0]].setup(length - 3);
            //  for (int i = 0; i < length - 3; i++)
          //    {
        //        controllers[params[0]].setId(i, params[i + 1]);
      //        }
        //esto es pa diff drive!!! PASO    }
            break;

          case ARB_CONTROL_WRITE:              // Write values to a controller
            statusPacket(id, 0);
            //esto es pa diff drive!!! PASO
//            if (params[0] < CONTROLLER_COUNT)
//            {
//              for (int i = 0; i < length - 4; i += 2)
//              {
//                controllers[params[0]].setNextPose(
//                    controllers[params[0]].getId(i / 2),
//                    params[i + 1] + (params[i + 2] << 8));
//              }
//              controllers[params[0]].readPose();
//              controllers[params[0]].interpolateSetup(params[length - 3] * 33);
//            }
            break;

          case ARB_CONTROL_STAT:               // Read status of a controller
            if (params[0] < CONTROLLER_COUNT)
            {
              SerialUSB.write((unsigned char) 0xff);
              SerialUSB.write((unsigned char) 0xff);
              SerialUSB.write((unsigned char) id);
              SerialUSB.write((unsigned char) 3);
              SerialUSB.write((unsigned char) 0);
              checksum = bioloid.interpolating();
              SerialUSB.write((unsigned char) checksum);
              checksum += id + 3;
              SerialUSB.write((unsigned char) 255 - ((checksum) % 256));
            }
            break;
          }
        }
        else if (id == 0xFE)
        {
          // sync read or write
          if (ins == ARB_SYNC_READ)
          {
            int start = params[0];    // address to read in control table
            int bytes = params[1];    // # of bytes to read from each servo
            int k = 2;
            checksum = id + (bytes * (length - 4)) + 2;
            SerialUSB.write((unsigned char) 0xff);
            SerialUSB.write((unsigned char) 0xff);
            SerialUSB.write((unsigned char) id);
            SerialUSB.write((unsigned char) 2 + (bytes * (length - 4)));
            SerialUSB.write((unsigned char) 0);     // error code
            // send actual data
            for (k = 2; k < length - 2; k++)
            {
//              if (ax12GetRegister(params[k], start, bytes) >= 0)
//              {
//                for (i = 0; i < bytes; i++)
//                {
//                  checksum += ax_rx_buffer[5 + i];
//                  SerialUSB.write((unsigned char) ax_rx_buffer[5 + i]);
//                }
//              }
//              else
//              {
                for (i = 0; i < bytes; i++)
                {
                  checksum += 255;
                  SerialUSB.write((unsigned char) 255);
                }
//              }
            }
            SerialUSB.write((unsigned char) 255 - ((checksum) % 256));
          }
          else
          {
//            // TODO: sync write pass thru
//            int k;
//            setTXall();
//            ax12write(0xff);
//            ax12write(0xff);
//            ax12write (id);
//            ax12write (length);
//            ax12write (ins);
//            for (k = 0; k < length; k++)
//              ax12write (params[k]);
            // no return
          }
        }
        else
        { // ID != 253, pass thru 
          switch (ins)
          {
          // TODO: streamline this
          case AX_READ_DATA:
          
  
  blink(6);
           // ax12GetRegister(id, params[0], params[1]);
            // return a packet: FF FF id Len Err params check
//            if (ax_rx_buffer[3] > 0)
//            {
//              for (i = 0; i < ax_rx_buffer[3] + 4; i++)
//                SerialUSB.write(ax_rx_buffer[i]);
//            }
//            ax_rx_buffer[3] = 0;
            break;

          case AX_WRITE_DATA:
//            if (length == 4)
//            {
//              ax12SetRegister(id, params[0], params[1]);
//            }
//            else
//            {
//              int x = params[1] + (params[2] << 8);
//              ax12SetRegister2(id, params[0], x);
//            }
            statusPacket(id, 0);
            break;

          }
        }
      }
    } // end mode == 5
  } // end while(available)
  // update joints
  bioloid.interpolateStep();
}

/////IDEA:  Intentar, pero.... si esto da mucho x culo,  xq no uso pypose.ino y añado la interface de PyPose a arbotix driver???

