import subprocess
import time

bashCommand = "./a.out /dev/ttyUSB0"

while(1):
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    time.sleep(5)
    



