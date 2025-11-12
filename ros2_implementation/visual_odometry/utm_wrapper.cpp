#include "LatLong-UTMconversion.h"
#include <cstring>

extern "C" {
    void latlon_to_utm_c(int reference_ellipsoid, double lat, double lon, 
                         double* utm_northing, double* utm_easting, char* utm_zone) {
        LLtoUTM(reference_ellipsoid, lat, lon, *utm_northing, *utm_easting, utm_zone);
    }
}