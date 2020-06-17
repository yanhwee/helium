from math import sin, cos, asin, sqrt, radians, degrees
from functools import wraps

def haversine(angle):
    return (sin(angle / 2)) ** 2
    # return (1 - cos(angle)) / 2

def haversineDist(latA, lonA, latB, lonB, r=6371e3):
    dLat = (latB - latA) / 2
    dLon = (lonB - lonA) / 2
    dist = 2 * r * asin(
        sqrt(haversine(dLat) + haversine(dLon) * cos(latA) * cos(latB)))
    return dist

def haversineLatLon(latA, lonA, y, x, r=6371e3):
    latB = x / r + latA
    lonB = asin(sin(y / 2 / r) / cos(latA)) * 2 + lonA
    return latB, lonB

def haversineDistDeg(latA, lonA, latB, lonB, r=6371e3):
    return haversineDist(radians(latA), radians(lonA), radians(latB), radians(lonB), r=r)

def haversineLatLonDeg(latA, lonA, y, x, r=6371e3):
    lat, lon = haversineLatLon(radians(latA), radians(lonA), y, x, r=r)
    return degrees(lat), degrees(lon)

def partial_map(func, iters, **kwargs):
    keys = [key for key in func.__code__.co_varnames if key not in kwargs.keys()]
    @wraps(func)
    def wrapper(*values):
        return func(zip(keys, values), **kwargs)
    return list(map(wrapper, iters))