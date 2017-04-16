from lib.mBot import *
import os, sys
import pygame
from pygame.locals import *

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

GRID_SPACE = 280

def setX(newX):
    x=newX
    
def setY(newY):
    y=newY
    
def setHeading(newHeading):
    x=newHeading
    
def setTravel(newTravel):
    tgtTravel=newTravel
    
def setTurn(newTurn):
    tgtTurn = newTurn  

def setUltrasonic(newUltrasonic):
    ultrasonic = newUltrasonic
    
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

x=0
y=0
heading=0
tgtTravel=0
tgtTurn = 0
ultrasonic = 0

turn=0
speed = 0

running = True

readoutFont = pygame.font.Font(None, 36)

#Initialise position and heading
bot.doGridX(123)
bot.doGridY(456)

while running:

    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == KEYDOWN:
            
            if event.key == K_ESCAPE:
                running = False
            if event.key == K_UP:
                speed = 100
                turn = 0

                tgtTravel = tgtTravel + GRID_SPACE
                bot.doGridTravel(tgtTravel)
                
            if event.key == K_DOWN:
                speed = -100
                turn = 0

                tgtTravel = tgtTravel - GRID_SPACE
                if tgtTravel < 0:
                    tgtTravel = 0                
                bot.doGridTravel(tgtTravel)                
                
            if event.key == K_RIGHT:
                speed = 100
                turn = 1
                
                tgtTurn = tgtTurn + 90
                bot.doGridTurn(tgtTurn)
                
            if event.key == K_LEFT:
                speed = 100
                turn = -1
               
                tgtTurn = tgtTurn - 90
                bot.doGridTurn(tgtTurn)
                        
        elif event.type == KEYUP:
            speed = 0
            turn = 0
                
        elif event.type == MOUSEBUTTONDOWN:
            pass
        elif event.type == MOUSEBUTTONUP:
            pass 
 
    #print "updating pos/heading..."            
    bot.requestGridX(setX)
    bot.requestGridY(setY)
    bot.requestGridHeading(setHeading)
    #print "updating turn/travel..."            
    bot.requestGridTurn(setTurn)
    bot.requestGridTravel(setTravel)
    #print "updating ultrasonic..."            
    ultrasonic = bot.requestUltrasonicSensor(3)

    displaySurf.blit(background, (0, 0))

    text = readoutFont.render("X : " + str(x), 1, TXTCOLOR)
    textpos = (50,50,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Y : " + str(y), 1, TXTCOLOR)
    textpos = (50,100,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Heading : " + str(heading), 1, TXTCOLOR)
    textpos = (50,150,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Target Travel Distance : " + str(tgtTravel), 1, TXTCOLOR)
    textpos = (50,200,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Target Turn Angle : " + str(tgtTurn), 1, TXTCOLOR)
    textpos = (50,250,0,0)
    displaySurf.blit(text, textpos)

    text = readoutFont.render("Ultrasonic Distance : " + str(ultrasonic), 1, TXTCOLOR)
    textpos = (50,300,0,0)
    displaySurf.blit(text, textpos)

    pygame.display.flip()
            
pygame.display.quit()
pygame.quit()
