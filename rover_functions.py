import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)


def readPos():
    try:
        state = ser.readline()
        state_conv = state.decode("utf-8")
        pos = parsePos(state_conv)
        #print(state_conv)
        return pos
    except:
        pass

def parsePos(state_conv):
    posType = state_conv.split(',')
    if (posType[0] == "$GPRMC"):
        # pos = posType[1].split(',')
        # print(pos)
        timeRead = posType[1]
        validBool = posType[2]
        # print(validBool)
        if (validBool) == "V":
            print("No Connection to Satellites at", timeRead)
            # print(timeRead)
            return 0
        elif(validBool == "A"):
            latitude = posType[3]
            lat_dir = posType[4]
            longitude = postype[5]
            long_dir = posType[6]
            roverSpeed = posType[7]
        return [latitude, lat_dir, longitude, long_dir, roverSpeed]

    elif(posType[0] == "$GPGGA"):
        time_ping= posType[1]
        latitude = posType[2]
        lat_dir = posType[3]
        longitude = posType[4]
        long_dir = posType[5]
        return [time_ping, latitude, lat_dir, longitude, long_dir]
def standby():
    time.sleep(5)
    #return True