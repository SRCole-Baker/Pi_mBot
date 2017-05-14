from lib.mBot import *
import os, sys
import pygame
from pygame.locals import *
import navigator

#             R    G    B
WHITE     = (255, 255, 255)
BLACK     = (  0,   0,   0)
RED       = (255,   0,   0)
BLUE      = (  0,   0, 255)
GREEN     = (  0, 255,   0)
DARKGREEN = (  0, 155,   0)
DARKGRAY  = ( 40,  40,  40)

BGCOLOR = DARKGRAY
TXTCOLOR = BLUE

GRID_SPACE = 300

x = 0
y = 0
heading = 0
tgtTravel = 0
tgtTurn = 0
ultrasonic = 0


def setX(newX):
    global x
    x = newX
    
def setY(newY):
    global y
    y = newY
    
def setHeading(newHeading):
    global heading
    heading = newHeading
    
def setTravel(newTravel):
    global tgtTravel
    tgtTravel = newTravel
    
def setTurn(newTurn):
    global tgtTurn
    tgtTurn = newTurn  

def setUltrasonic(newUltrasonic):
    global ultrasonic
    ultrasonic = newUltrasonic

def onRxGridData():
    pass
    
bot = mBot()
try:
    bot.startWithSerial("/dev/ttyUSB0")
except:
    bot.startWithSerial("/dev/ttyACM0")

#bot.startWithHID()

pygame.init()
#FPSCLOCK = pygame.time.Clock()
displaySurf = pygame.display.set_mode((640, 480))
pygame.display.set_caption('mBot Control')

background = pygame.Surface(displaySurf.get_size())
background = background.convert()
background.fill(BGCOLOR)

#Opening USB serial port will reset the arduino - wait for it to reboot before continuing
sleep(3)



turn=0
speed = 0

running = True

readoutFont = pygame.font.Font(None, 36)

#Initialise position and heading
bot.doGridX(123)
bot.doGridY(456)
bot.doGridHeading(90)

commandThread = threading.Thread(target=navigator.navigator, args=(bot,))
commandThread.start()
#navigator.navigator()

while running:

    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == KEYDOWN:
            
            if event.key == K_ESCAPE:
                running = False
            if event.key == K_UP:

                tgtTravel = tgtTravel + GRID_SPACE
                bot.doGridTravel(GRID_SPACE)
                 #tgtTravel = bot.requestGridTravel()
                
            if event.key == K_DOWN:
                
                navigator.doSquare()
                
                tgtTravel = tgtTravel - GRID_SPACE
                if tgtTravel < 0:
                    tgtTravel = 0                
                bot.doGridTravel(0)
                #tgtTravel = bot.requestGridTravel()
                
            if event.key == K_RIGHT:
                
                tgtTurn = bot.gridTurn.value + 90
                bot.doGridTurn(tgtTurn)
                #tgtTurn = bot.requestGridTurn()
                
            if event.key == K_LEFT:
                
                tgtTurn = bot.gridTurn.value - 90
                bot.doGridTurn(tgtTurn)
                #tgtTurn = bot.requestGridTurn()
                
            if event.key == K_F1:
                navigator.doSquare()                
                
            if event.key == K_SPACE:
                navigator.stop()
                
                bot.doGridTurn(0)
                bot.doGridTravel(0)            
                        
        elif event.type == KEYUP:
            pass
        
        elif event.type == MOUSEBUTTONDOWN:
            pass
        elif event.type == MOUSEBUTTONUP:
            pass 
 
    #print "updating pos/heading..."            
#    bot.requestGridX(setX)
#    bot.requestGridY(setY)
#    bot.requestGridHeading(setHeading)
    #bot.requestLineFollower(2, setHeading)
    #print "updating turn/travel..."            
#    bot.requestGridTurn(setTurn)
#    bot.requestGridTravel(setTravel)
    #print "updating ultrasonic..."            
#    bot.requestUltrasonicSensor(3, setUltrasonic)

    bot.requestGridData(onRxGridData)

    #print bot.gridStatus.ready

    displaySurf.blit(background, (0, 0))

    text = readoutFont.render("X : " + str(bot.gridX.value), 1, TXTCOLOR)
    textpos = (50,50,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Y : " + str(bot.gridY.value), 1, TXTCOLOR)
    textpos = (50,100,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Heading : " + str(bot.gridHeading.value), 1, TXTCOLOR)
    textpos = (50,150,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Target Travel Distance : " + str(bot.gridTravel.value), 1, TXTCOLOR)
    textpos = (50,200,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Busy Flag : " + str(bot.gridStatus.busy), 1, TXTCOLOR)
    textpos = (50,250,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Ultrasonic Distance : " + str(bot.ultrasonicSensor.value), 1, TXTCOLOR)
    textpos = (50,300,0,0)
    displaySurf.blit(text, textpos)

    pygame.display.flip()

navigator.quit()           
pygame.display.quit()
pygame.quit()
bot.exit()
