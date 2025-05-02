class Waypoint :
    lat = 0.0 # Latitude in relative coordinates
    lon = 0.0 # Longitude in relative coordinates
    alt = 0.0 # Altitude in relative coordinates
    
    def __init__(self, lat, lon, alt):  # <- Correct constructor
            self.lat = lat
            self.lon = lon
            self.alt = alt