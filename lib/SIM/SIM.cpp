/*
  GSM.cpp
  2013 Copyright (c) Seeed Technology Inc.  All right reserved.
 
  Author:lawliet.zou@gmail.com
  2013-11-14
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
 
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
 
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <mbed.h>
#include "SIM.h"

namspace SIM{

    namespace{
        
    }
    int GSM::readBuffer(char *buffer,int count)
    {
        int i = 0;
        timeCnt.start();  // start timer
        while(1) {
            while (gprsSerial.readable()) {
                char c = gprsSerial.getc();
                if (c == '\r' || c == '\n') c = '$';
                buffer[i++] = c;
                if(i > count)break;
            }
            if(i > count)break;
            if(timeCnt.read() > DEFAULT_TIMEOUT) {
                timeCnt.stop();
                timeCnt.reset();
                break;
            }
        }
        wait(0.5);
        while(gprsSerial.readable()) {  // display the other thing..
            char c = gprsSerial.getc();
        }
        return 0;
    }
    
    void cleanBuffer(char *buffer, int count)
    {
        for(int i=0; i < count; i++) {
            buffer[i] = '\0';
        }
    }
    
    void GSM::sendCmd(char *cmd)
    {
        gprsSerial.puts(cmd);
    }
    
    int GSM::waitForResp(char *resp, int timeout)
    {
        int len = strlen(resp);
        int sum=0;
        timeCnt.start();
    
        while(1) {
            if(gprsSerial.readable()) {
                char c = gprsSerial.getc();
                sum = (c==resp[sum]) ? sum+1 : 0;
                if(sum == len)break;
            }
            if(timeCnt.read() > timeout) {  // time out
                timeCnt.stop();
                timeCnt.reset();
                return -1;
            }
        }
        timeCnt.stop();                 // stop timer
        timeCnt.reset();                    // clear timer
        while(gprsSerial.readable()) {      // display the other thing..
            char c = gprsSerial.getc();
        }
    
        return 0;
    }
    
    int GSM::sendCmdAndWaitForResp(char *cmd, char *resp, int timeout)
    {
        sendCmd(cmd);
        return waitForResp(resp,timeout);
    }
    
    int GSM::powerCheck(void)
    {
        return sendCmdAndWaitForResp("AT\r\n", "OK", 2);    
    }
    
    int GSM::init(void)
    {
        for(int i = 0; i < 3; i++){
            sendCmdAndWaitForResp("AT\r\n", "OK", DEFAULT_TIMEOUT);
            wait(0.5);
        }
        if(0 != checkSIMStatus()) {
            return -1;
        }
        if(checkSignalStrength()<1) {
            return -1;
        }
        if(0 != settingSMS()) {
            return -1;
        }
        return 0;
    }
    
    int GSM::checkSIMStatus(void)
    {
        char gprsBuffer[30];
        int count = 0;
        cleanBuffer(gprsBuffer,30);
        while(count < 3) {
            sendCmd("AT+CPIN?\r\n");
            readBuffer(gprsBuffer,30);
            if((NULL != strstr(gprsBuffer,"+CPIN: READY"))) {
                break;
            }
            count++;
            wait(1);
        }
    
        if(count == 3) {
            return -1;
        }
        return 0;
    }
    
    int GSM::checkSignalStrength(void)
    {
        char gprsBuffer[100];
        int index,count = 0;
        cleanBuffer(gprsBuffer,100);
        while(count < 3) {
            sendCmd("AT+CSQ\r\n");
            readBuffer(gprsBuffer,25);
            if(sscanf(gprsBuffer, "AT+CSQ$$$$+CSQ: %d", &index)>0) {
                break;
            }
            count++;
            wait(1);
        }
        if(count == 3) {
            return -1;
        }
        return index;
    }
    
    int GSM::settingSMS(void)
    {
        if(0 != sendCmdAndWaitForResp("AT+CNMI=2,2\r\n", "OK", DEFAULT_TIMEOUT)) {
            return -1;
        }
        if(0 != sendCmdAndWaitForResp("AT+CMGF=1\r\n", "OK", DEFAULT_TIMEOUT)) {
            return -1;
        }
        return 0;
    }
    
    int GSM::sendSMS(char *number, char *data)
    {
        char cmd[64];
        while(gprsSerial.readable()) {
            char c = gprsSerial.getc();
        }
        snprintf(cmd, sizeof(cmd),"AT+CMGS=\"%s\"\r\n",number);
        if(0 != sendCmdAndWaitForResp(cmd,">",DEFAULT_TIMEOUT)) {
            return -1;
        }
        wait(1);
        gprsSerial.puts(data);
        gprsSerial.putc((char)0x1a);
        return 0;
    }
    
    int GSM::readSMS(char *message, int index)
    {
        int i = 0;
        char gprsBuffer[100];
        char *p,*s;
        gprsSerial.printf("AT+CMGR=%d\r\n",index);
        cleanBuffer(gprsBuffer,100);
        readBuffer(gprsBuffer,100);
        if(NULL == ( s = strstr(gprsBuffer,"+CMGR"))) {
            return -1;
        }
        if(NULL != ( s = strstr(gprsBuffer,"+32"))) {
            p = s + 6;
            while((*p != '$')&&(i < SMS_MAX_LENGTH-1)) {
                message[i++] = *(p++);
            }
            message[i] = '\0';
        }
        return 0;
    }
    
    int GSM::deleteSMS(int index)
    {
        char cmd[32];
        snprintf(cmd,sizeof(cmd),"AT+CMGD=%d\r\n",index);
        sendCmd(cmd);
        return 0;
    }
    
    int GSM::getSMS(char* message)
    {
        if(NULL != messageBuffer) {
            strncpy(message,messageBuffer,SMS_MAX_LENGTH);
        }
        return 0;
    }
    
    int GSM::callUp(char *number)
    {
        if(0 != sendCmdAndWaitForResp("AT+COLP=1\r\n","OK",5)) {
            return -1;
        }
        wait(1);
        gprsSerial.printf("\r\nATD%s;\r\n",NULL==number?phoneNumber:number);
        return 0;
    }
    
    int GSM::answer(void)
    {
        gprsSerial.printf("ATA\r\n");
        return 0;
    }
    
    int GSM::loopHandle(void)
    {
        char gprsBuffer[100];
        int i;
        char *s = NULL;
        while(gprsSerial.readable()) {
            char c = gprsSerial.getc();
        }
        wait(0.5);
    START:
        cleanBuffer(gprsBuffer,100);
        i = 0;
        while(1) {
            if(gprsSerial.readable()) {
                timeCnt.start();  // start timer
                while(1) {
                    while (gprsSerial.readable()) {
                        char c = gprsSerial.getc();
                        if (c == '\r' || c == '\n') c = '$';
                        gprsBuffer[i] = c;
                        i++;
                        if(i > 100) {
                            i = 0;
                            break;
                        }
                    }
                    if(timeCnt.read() > 2) {          // time out
                        timeCnt.stop();
                        timeCnt.reset();
                        break;
                    }
                }
                break;
            }
        }
        if(NULL != strstr(gprsBuffer,"RING")) {
            return MESSAGE_RING;
        } else if(NULL != (s = strstr(gprsBuffer,"+CMT"))) { //SMS: $$+CMTI: "SM",24$$
            if(NULL != (s = strstr(gprsBuffer,"+32"))) {
                s += 6;
                int i = 0;
                cleanBuffer(messageBuffer,SMS_MAX_LENGTH);
                while((*s != '$')&&(i < SMS_MAX_LENGTH-1)) {
                    messageBuffer[i++] = *(s++);
                }
                messageBuffer[i] = '\0';
                return MESSAGE_SMS;
            } else {
                goto START;
            }
        } else {
            goto START;
        }
    }
    
    int GSM::networkInit(char* apn, char* userName, char* passWord)
    {
        char cstt[64];
        snprintf(cstt,sizeof(cstt),"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n",apn,userName,passWord);
        if(0 != sendCmdAndWaitForResp(cstt, "OK", DEFAULT_TIMEOUT)) {
            return -1;
        }
        return 0;
    }
    
    int GSM::connectTCP(char *ip, char *port)
    {
        char cipstart[64];
    #if 0
        if(0 != sendCmdAndWaitForResp("AT+CSTT=\"CMNET\",\"\",\"\"\r\n", "OK", 5)) {
            return -1;
        }
    #endif
        sprintf(cipstart, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n", ip, port);
        if(0 != sendCmdAndWaitForResp(cipstart, "OK", DEFAULT_TIMEOUT)) {
            return -1;
        }
        return 0;
    }
    int GSM::sendTCPData(char *data)
    {
        char cmd[64];
        int len = strlen(data);
        snprintf(cmd,sizeof(cmd),"AT+CIPSEND=%d\r\n",len);
        if(0 != sendCmdAndWaitForResp(cmd,">",DEFAULT_TIMEOUT)) {
            return -1;
        }
        if(0 != sendCmdAndWaitForResp(data,"OK",DEFAULT_TIMEOUT)) {
            return -1;
        }
        return 0;
    }
    
    int GSM::closeTCP(void)
    {
        sendCmd("AT+CIPCLOSE\r\n");
        return 0;
    }
    
    int GSM::shutTCP(void)
    {
        sendCmd("AT+CIPSHUT\r\n");
        return 0;
    }

}

 

 