import os
import sys
import time

running = True

driveSquare = False

def quit():
    global running
    running = False

def doSquare():
    global driveSquare
    driveSquare = True

def stop():
    global driveSquare
    driveSquare = False
    
def botMove(bot, distance):
    bot.doGridTravel(distance)
    bot.gridStatus.cancelCurrentReq(bot)
    bot.gridStatus.busy = True;
    while bot.gridStatus.busy:
        time.sleep(0.1)

def botTurn(bot, angle):
    bot.doGridTurn(angle)
    bot.gridStatus.cancelCurrentReq(bot)
    bot.gridStatus.busy = True;
    while bot.gridStatus.busy:
        time.sleep(0.1)    

def navigator(bot):
    while running:
        
        if driveSquare:
            for i in range(3):
                if driveSquare:
                    botMove(bot, 300)
                if driveSquare:
                    botTurn(bot, 90)
            driveSquare = 0

        time.sleep(0.1)

    bot.doGridTravel(0)
    bot.doGridTurn(0)
