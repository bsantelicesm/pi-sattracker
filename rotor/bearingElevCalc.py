import math
import pyserial
import tkinter

class PolarCoordinate:
    earthRadius = 6371000

    def __init__(self, lat, lon, alt):
        self.latitude = math.radians(lat)
        self.longitude = math.radians(lon)
        self.altitude = alt

    def distanceTo(self, point):
        a = math.pow(math.sin((self.latitude - point.latitude)/2), 2) + math.cos(self.longitude) * math.cos(point.longitude) * math.pow(math.sin((self.longitude - point.lognitude)/2), 2)
        b = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = earthRadius * b
        return distance

    def azimuthTo(self, point):
        azimuth = math.atan2(math.sin(point.longitude - self.longitude) * math.cos(point.latitude), math.sin(point.latitude) - math.sin(self.latitude) * math.cos(point.latitude * math) * math.cos(point.longitude - self.longitude))
        return math.degrees(azimuth)

    def commHopDistance(self, point):
        arcAngle = distanceTo(self, point) / earthRadius
        commHopDist = math.sqrt(math.pow(self.altitude + earthRadius, 2) + math.pow(point.altitude + earthRadius, 2) - 2 * (self.altitude + earthRadius) * (point.altitude + earthRadius) * math.cos(arcAngle))
        return commHopDist

    def elevationTo(self, point):
        arcAngle = distanceTo(self, point) / earthRadius
        c = commHopDistance(self, point)
        desiredVertex = asin((point.altitude + earthRadius) * math.sin(arcAngle) / c)
        return math.degrees(desiredVertex) - 90

station = PolarCoordinate(0, 0, 0)
sonde = PolarCoordinate(0, 0, 0)
