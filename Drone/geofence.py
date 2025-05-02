class GeoFence:
    def __init__(self, lat_min, lat_max, lon_min, lon_max, alt_max):
        self.lat_min = lat_min
        self.lat_max = lat_max

        self.lon_min = lon_min
        self.lon_max = lon_max
        
        self.alt_max = alt_max  

    def is_within_bounds(self, waypoint):
        return (
            self.lat_min <= waypoint.lat <= self.lat_max and
            self.lon_min <= waypoint.lon <= self.lon_max and
            waypoint.alt <= self.alt_max
        )
