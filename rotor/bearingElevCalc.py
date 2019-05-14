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
        a = math.pow(math.sin((self.latitude - point.latitude)/2), 2) + math.cos(self.longitude) * math.cos(point.longitude) * math.pow(math.sin((self.longitude - point.lognitude)/2), 2)
        b = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = earthRadius * b
        return distance

    def azimuthTo(self, point):  #calculates heading to a specific target. Also uses haversine function.
        azimuth = math.atan2(math.sin(point.longitude - self.longitude) * math.cos(point.latitude), math.sin(point.latitude) - math.sin(self.latitude) * math.cos(point.latitude * math) * math.cos(point.longitude - self.longitude))
        return math.degrees(azimuth)

    def commHopDistance(self, point): #calculates distance in a straight line between two
        arcAngle = distanceTo(self, point) / earthRadius
        commHopDist = math.sqrt(math.pow(self.altitude + earthRadius, 2) + math.pow(point.altitude + earthRadius, 2) - 2 * (self.altitude + earthRadius) * (point.altitude + earthRadius) * math.cos(arcAngle))
        return commHopDist

    def elevationTo(self, point):
        arcAngle = distanceTo(self, point) / earthRadius
        c = commHopDistance(self, point)
        desiredVertex = asin((point.altitude + earthRadius) * math.sin(arcAngle) / c)
        return math.degrees(desiredVertex) - 90

class EasyComm:

    def __init__(self, serialPort):
        self.UART = serial.Serial(port=serialPort)

    def send(azimuth, elevation):
        azString = '%05.1f' % azimuth
        elString = '%05.1f' % elevation
        command = "AZ=" + azString + " EL=" + elString + "\n"
        self.UART.write(command)

    def getAngles():
        self.UART.write("AZ EL\n")

class APRS:
    def __init__(self, callsign, target):
        self.server = aprslib.IS(callsign)
        self.target = target

    def callback(self, text):
        if "to" in text:
            if self.target in text["to"]:
                return text

    def start(self):
        self.server.connect()
        self.server.consumer(callback)

station = PolarCoordinate(0, 0, 0)
sonde = PolarCoordinate(0, 0, 0)
tracker = EasyComm('COM2')
aprsis = APRS("CE3BUC")
print "SPEL Radiosonde Tracker Station Software DEV v0.1"
print "Initializing APRS Service"
