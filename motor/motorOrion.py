from megapi import *

def onRead(level):
    print level

if __name__ == '__main__':
    bot = MegaPi()
    #bot.start('/dev/ttyUSB0')
    bot.start()
    '''
    while(1):
        for i in range(100):
            print "Reading..."
            sleep(0.1)
            bot.digitalRead(18, onRead)
    '''
    
    bot.motorRun(1,0);
    sleep(1);
    while 1:
	    sleep(1);
	    bot.motorRun(1,250);
	    sleep(1);
	    bot.motorRun(1,0);
	    sleep(1);
	    bot.motorRun(1,-250);
	    sleep(1);
	    bot.motorRun(1,0);
    
