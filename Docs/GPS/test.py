import math
import cv2
import numpy as np

RAD2DEG=180.0/math.pi
DEG2RAD=math.pi/180.0
imScale = 0.5
image = np.zeros((int(1400*imScale),int(1400*imScale),3), np.uint8)
def distAndHeadingPlane( currentLat, currentLon, targetLat, targetLon):
    RAD2DEG=180.0/math.pi
    dlat = (targetLat-currentLat)
    dlon = (targetLon-currentLon)
    angleF = math.atan2(dlon,dlat)*1e5*RAD2DEG;# in 1e-5 degrees [-18000000;18000000]
    distF = math.sqrt(dlon*dlon+dlat*dlat)
    return int(distF),int(angleF)

def displayScene( wps, pos, angle):
        
        image[:] = (255,255,255)
        for i,wp in enumerate(wps):
                cv2.line(image,(int((wps[i-1][1]/1000+100)*imScale),int((1400-wps[i-1][0]/1000-100)*imScale)),(int((wps[i][1]/1000+100)*imScale),int((1400-wps[i][0]/1000-100)*imScale)),(0,0,0),1)
                if i == targetNum:
                        color=(0,255,0)
                else:
                        color=(255,0,0)
                cv2.circle(image,(int((wp[1]/1000+100)*imScale),int((1400-wp[0]/1000-100)*imScale)),int(5*imScale),color,-1)

        cp = (int((pos[1]/1000+100)*imScale),int((1400-pos[0]/1000-100)*imScale))
        cp2 = (int(cp[0]+10*math.sin(angle*DEG2RAD)),int(cp[1]-10*math.cos(angle*DEG2RAD)))
        cv2.circle(image,cp,int(5*imScale),(255, 128, 0),-1)
        cv2.line(image,cp,cp2,(255,128,0),1)
        cv2.imshow('render',image)
        cv2.waitKey(0)



#waypoints in an arbitrary plane,
#[Latitude,longitue] in mm (north is at [inf,0])
wps = [[0,0],
        [1000000,0],
        [1200000,900000],
        [500000,1150000]]

for i in range(1,len(wps)):
    d,a = distAndHeadingPlane(wps[i-1][0],wps[i-1][1],wps[i][0],wps[i][1])
    print((d,a))

Dt=0.01 #second
maxTurnRate = 10.0 # °/s
trunRate=0.0

gpsUpdateCount = 100 # refresh period of gps in timesteps

wind=[0*278,70*278] #mm/s

pos = wps[0].copy()
orient = 0#physical orientation of the nose of the craft

airSpeed = 75*278#conversion from km/h to mm/s
lastGPSPos = pos.copy()

targetNum=1
_,neededHeading = distAndHeadingPlane(wps[targetNum-1][0],wps[targetNum-1][1],wps[targetNum][0],wps[targetNum][1])

stepCount=0
while True:

        if stepCount%gpsUpdateCount == 0:
                d,a = distAndHeadingPlane(pos[0],pos[1],wps[targetNum][0],wps[targetNum][1])
                if d<20000:
                        targetNum= (targetNum+1)%len(wps)
                        _,neededHeading = distAndHeadingPlane(wps[targetNum-1][0],wps[targetNum-1][1],wps[targetNum][0],wps[targetNum][1])
                        d,a = distAndHeadingPlane(pos[0],pos[1],wps[targetNum][0],wps[targetNum][1])
                dc,ac = distAndHeadingPlane(lastGPSPos[0],lastGPSPos[1],pos[0],pos[1])
                gpsSpeed = dc/(Dt*gpsUpdateCount)

                displayScene(wps,pos,orient)

                drift = neededHeading-a
                while(drift>17999999):
                    drift-=36000000
                while(drift<-18000000):
                    drift+=36000000
                
                if (drift>-6000000 and drift<6000000):
                    #algo : 
                    # wantedDir = targetDir-drift -> to have the wanted direction such that we cut the remaining segment in two (maybe do a bit more than that to comm quicker to the path?)
                    # error= current -wantedDir;
                    error=ac-(a-d*5e-6*drift)
                else:
                    # drifted too much from trajectory, simple target tracking.
                    error=ac-a
                while(error>17999999):
                    error-=36000000
                while(error<-18000000):
                    error+=36000000
                print(error)
                trunRate = min([max([-maxTurnRate,-error]),maxTurnRate])
                lastGPSPos=pos.copy()


        pos[0]+=Dt*(airSpeed*math.cos(orient*DEG2RAD)+wind[0])
        pos[1]+=Dt*(airSpeed*math.sin(orient*DEG2RAD)+wind[1])
        orient+=Dt*trunRate
        stepCount+=1


