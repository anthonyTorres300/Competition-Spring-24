class waypoint :
    lat = 0.0 # Latitude in relative coordinates
    lon = 0.0 # Longitude in relative coordinates
    alt = 0.0 # Altitude in relative coordinates

    def _init_(self, lat, lon, alt): # Constructor for waypoint object
        self.lat = lat 
        self.lon = lon
        self.alt = alt