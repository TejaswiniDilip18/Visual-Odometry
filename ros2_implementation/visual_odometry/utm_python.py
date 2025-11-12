import ctypes
import os
import numpy as np
from ctypes import c_double, c_int, c_char_p, POINTER, byref

# Load the shared library
# lib_path = os.path.join(os.path.dirname(__file__), 'libutm_converter.so')
lib_path = '/root/ros2_docker_ws/src/visual_odometry/visual_odometry/libutm_converter.so'
if not os.path.exists(lib_path):
    lib_path = './libutm_converter.so'  # Try current directory
    
try:
    utm_lib = ctypes.CDLL(lib_path)
except OSError as e:
    print(f"Error loading library: {e}")
    print(f"Make sure libutm_converter.so is in the current directory or same folder as this script")
    utm_lib = None

# Define the function signature
if utm_lib:
    utm_lib.latlon_to_utm_c.argtypes = [
        c_int,           # reference_ellipsoid
        c_double,        # lat
        c_double,        # lon  
        POINTER(c_double), # utm_northing
        POINTER(c_double), # utm_easting
        c_char_p         # utm_zone
    ]
    utm_lib.latlon_to_utm_c.restype = None

def LLtoUTM(reference_ellipsoid, lat, lon):
    """
    Convert latitude/longitude to UTM coordinates
    Returns: (utm_northing, utm_easting, utm_zone)
    """
    if utm_lib is None:
        raise RuntimeError("UTM library not loaded")
        
    utm_northing = c_double()
    utm_easting = c_double()
    utm_zone = ctypes.create_string_buffer(10)
    
    utm_lib.latlon_to_utm_c(
        reference_ellipsoid,
        c_double(lat),
        c_double(lon),
        byref(utm_northing),
        byref(utm_easting),
        utm_zone
    )
    
    return utm_northing.value, utm_easting.value, utm_zone.value.decode('utf-8')

def get_absolute_scale(frame_id, oxts_data_path):
    """
    Get the absolute scale of the trajectory from GPS data
    Python implementation of the C++ getAbsoluteScale function
    """
    import os
    import math
    
    # Construct file paths
    filename = os.path.join(oxts_data_path, f"{frame_id:010d}.txt")
    filename_prev = os.path.join(oxts_data_path, f"{frame_id-1:010d}.txt")
    
    lats, lons, alts = [], [], []
    utm_northings, utm_eastings = [], []
    
    # Read GPS Data from both files
    try:
        # Read current frame
        if os.path.exists(filename):
            with open(filename, 'r') as f:
                line = f.readline().strip()
                if line:
                    values = line.split()
                    if len(values) >= 3:
                        lat, lon, alt = float(values[0]), float(values[1]), float(values[2])
                        lats.append(lat)
                        lons.append(lon) 
                        alts.append(alt)
        else:
            print(f"File not found: {filename}")
            return -1
            
        # Read previous frame
        if os.path.exists(filename_prev):
            with open(filename_prev, 'r') as f:
                line = f.readline().strip()
                if line:
                    values = line.split()
                    if len(values) >= 3:
                        lat, lon, alt = float(values[0]), float(values[1]), float(values[2])
                        lats.append(lat)
                        lons.append(lon)
                        alts.append(alt)
        else:
            print(f"File not found: {filename_prev}")
            return -1
            
    except Exception as e:
        print(f"Error reading GPS files: {e}")
        return -1
    
    if len(lats) < 2:
        print("Insufficient GPS data")
        return -1
    
    # Convert GPS coordinates to UTM
    reference_ellipsoid = 23  # WGS-84
    
    try:
        for i in range(len(lats)):
            utm_northing, utm_easting, utm_zone = LLtoUTM(reference_ellipsoid, lats[i], lons[i])
            utm_northings.append(utm_northing)
            utm_eastings.append(utm_easting)
    except Exception as e:
        print(f"Error in UTM conversion: {e}")
        return -1
    
    # Calculate the distance between the first and last UTM coordinates
    dx = utm_eastings[-1] - utm_eastings[0]  # Change in easting
    dy = utm_northings[-1] - utm_northings[0]  # Change in northing
    dz = alts[-1] - alts[0]  # Change in altitude
    
    # Calculate 3D Euclidean distance
    gps_scale = math.sqrt(dx * dx + dy * dy + dz * dz)
    
    return gps_scale