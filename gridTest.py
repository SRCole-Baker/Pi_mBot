from lib.mBot import *

print "Starting..."
bot = mBot()
print "Started"
#bot.startWithSerial("/dev/ttyUSB0")
#bot.startWithHID()
bot.startWithSerial("/dev/ttyACM0")

sensorData = []
for i in range (0, 3):
        sensorData.append(0)

GRID_X_IDX = 0
GRID_Y_IDX = 1
        
bot.setSensorData(sensorData)     

while(1):
        try:                
                #bot.doMove(100,100)
                #print "run forward"
                #sleep(2)
                #bot.doMove(-100,-100)
                #print "run backward"
                #sleep(2)
                #bot.doMove(0,0)
                #print "stop"

                print "Writing Coordinates..."
                
                bot.doGridX(11)
                sleep(2)
                bot.doGridY(22)                
                sleep(2)
                print "Reading coordinates..."
                bot.requestGridX(GRID_X_IDX)
                sleep(2)
                bot.requestGridY(GRID_Y_IDX)
                sleep(2)
                #print "Sensor Data X:" + str(sensorData[GRID_X_IDX])
                #print "Sensor Data Y:" + str(sensorData[GRID_Y_IDX])

                print "Sensor Data X:" + str(bot.gridX.value)
                print "Sensor Data Y:" + str(bot.gridY.value)
        
        except Exception,ex:
                print str(ex)
