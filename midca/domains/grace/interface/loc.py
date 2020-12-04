from __future__ import division
import math
import numpy as np
import json


def LocalGrid2LatLong(dfEast, dfNorth):
    dfa  = 6378137
    dfb = 6356752
    LatOrig = 31.421064
    LongOrig = -80.921200
    dftanlat2 = pow(math.tan( np.deg2rad(LatOrig) ), 2 )
    dfRadius = dfb*math.sqrt(1+dftanlat2) / math.sqrt( ( pow(dfb,2)/pow(dfa,2) )+dftanlat2)


    dfYArcRad = math.asin( dfNorth/dfRadius )
    dfYArcDeg = np.rad2deg(dfYArcRad)

    dfXArcRad = math.asin( dfEast/( dfRadius* math.cos( np.deg2rad(LatOrig) ) ) )
    dfXArcDeg = np.rad2deg(dfXArcRad)


    dfLat = dfYArcDeg + LatOrig
    dfLon = dfXArcDeg + LongOrig

    return dfLat, dfLon

def LatLong2LocalGrid(lat, lon):
    dfa  = 6378137
    dfb = 6356752
    LatOrig = 31.421064
    LongOrig = -80.921200

    dftanlat2 = pow(math.tan( np.deg2rad(lat) ), 2 )
    dfRadius = dfb*math.sqrt(1+dftanlat2) / math.sqrt( ( pow(dfb,2)/pow(dfa,2) )+dftanlat2)


    dXArcDeg  = np.deg2rad((lon - LongOrig))
    dX = dfRadius * math.sin(dXArcDeg)*math.cos(np.deg2rad(lat))

    dYArcDeg  = np.deg2rad((lat - LatOrig))
    dY = dfRadius * math.sin(dYArcDeg)


    #This is the total distance traveled thus far, either North or East
    MetersNorth = dY
    MetersEast  = dX
    return dX, dY


def ConvertMultipleLatLongToWaypoints(points):
    way_points = []
    for point in points:
        x, y = LatLong2LocalGrid(point[0], point[1])
        way_points.append([x,y])
    return way_points

if __name__ == "__main__":
    lat, long = LocalGrid2LatLong(1200,-6000)
    x, y = LatLong2LocalGrid(lat, long)
    print ("latitude :" +str(lat) + " longitude :" +str(long))
    print ("x :" +str(x) + "y :" +str(y))

    data = {}
    lat, long = LocalGrid2LatLong(10,-6272)
    data["InitialCoordinates"] = [lat, long]
    lat, long = LocalGrid2LatLong(1200,-6000)
    data["DestCoordinates"] = [lat, long]

    with open('data.json', 'w') as outfile:
        json.dump(data, outfile)
