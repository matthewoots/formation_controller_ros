from pygeodesy.geoids import GeoidPGM

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

def geoid_height(lat, lon):
    """Calculates AMSL to ellipsoid conversion offset.
    Uses EGM96 data with 5' grid and cubic interpolation.
    The value returned can help you convert from meters 
    above mean sea level (AMSL) to meters above
    the WGS84 ellipsoid.

    If you wannt to go from AMSL to ellipsoid height, add the value.

    To go from ellipsoid height to AMSL, subtract this value.
    """
    return _egm96.height(lat, lon)

lat1 = 47.3977444;
long1 = 8.5456599;
lat0 = 47.3977418;
long0 = 8.5455935;

diff0 = geoid_height(lat0, long0)
diff1 = geoid_height(lat1, long1)

print("diff0 is %f",diff0)
print("diff0 is %f",diff1)
