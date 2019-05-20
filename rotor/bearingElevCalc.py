import math
import serial
import aprslib

class PolarCoordinate:  #object for storing and operating locations in 3D polar space.
    earthRadius = 6371000 #radius of the earth, for calculations

    def __init__(self, lat, lon, alt):
        self.latitude = math.radians(lat)
        self.longitude = math.radians(lon)
        self.altitude = alt  #define latitude, longitude and altitude variables.

    def distanceTo(self, point): #uses the haversine function for calculating the arc distance between two points.
        deltaLatHalf = math.radians((math.degrees(point.latitude) - math.degrees(self.latitude)) / 2)
        deltaLonHalf = math.radians((math.degrees(point.longitude) - math.degrees(self.longitude)) / 2)

        a = math.pow(math.sin(deltaLatHalf), 2) + (math.cos(self.latitude) * math.cos(point.latitude) * math.pow(math.sin(deltaLonHalf), 2))
        b = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = self.earthRadius * b
        return distance

    def azimuthTo(self, point):  #calculates heading to a specific target. Also uses haversine function.
        deltaLon = math.radians(math.degrees(point.longitude) - math.degrees(self.longitude))

        azi = math.atan2(math.sin(deltaLon) * math.cos(point.latitude), math.cos(self.latitude) * math.sin(point.latitude) - math.sin(self.latitude) * math.cos(point.latitude) * math.cos(deltaLon))
        return math.degrees(azi)

    def commHopDistance(self, point): #calculates distance in a straight line between two points.
        arcAngle = self.distanceTo(point) / self.earthRadius
        commHopDist = math.sqrt(math.pow(self.altitude + self.earthRadius, 2) + math.pow(point.altitude + self.earthRadius, 2) - 2 * (self.altitude + self.earthRadius) * (point.altitude + self.earthRadius) * math.cos(arcAngle))
        return commHopDist

    def elevationTo(self, point): #returns elevation between two points. NOT YET WORKING.
        arcAngle = self.distanceTo(point) / self.earthRadius
        c = self.commHopDistance(point)
        desiredVertex = math.asin(((point.altitude + self.earthRadius) * math.sin(arcAngle)) / c)
        return -(math.degrees(desiredVertex)  - 90)

class EasyComm: #manage connection between an EasyComm I antenna rotor, connected to a serial port.

    def __init__(self, serialPort):
        self.UART = serial.Serial(port=serialPort) #pyserial serial port object.
        self.measuredAzimuth
        self.measuredElevation #azimuth and elevation obtained from getAngles()

    def send(self, azimuth, elevation):
        azString = '%05.1f' % azimuth
        elString = '%05.1f' % elevation
        command = "AZ=" + azString + " EL=" + elString + "\n" #creates proper string in accordance to EasyComm I protocol
        self.UART.write(command) #sends to rotor.

    def getAngles(self): #gets current measured angles for the rotor.
        self.UART.write("AZ EL\n")
        serialIn = self.UART.read(self.UART.in_waiting()) #check buffer and get answer from rotor.
        az, el = serialIn.split(" ")
        self.measuredAzimuth = float(az[3:])
        self.measuredElevation = float(el[3:]) #parse data and return measured data.

class APRS: #gets data from APRS-IS servers and returns important data.

    def __init__(self, callsign, target):
        self.server = aprslib.IS(callsign) #aprslib internet services connection
        self.target = target

    def callback(self, text):
        if "to" in text:
            if self.target in text["to"]:
                return text

    def start(self):
        self.server.connect()
        self.server.consumer(callback)

print "SPEL Radiosonde Tracker Station Software DEV v0.1"
call = input("Your SSID: ")
tget = input("Target SSID: ")
print "Initializing APRS Service..."
aprsis = APRS(call, tget)
