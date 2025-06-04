import time
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
import firebase_admin
from firebase_admin import credentials, db
from math import radians, sin, cos, sqrt, atan2

# Initialize Firebase Admin SDK
cred = credentials.Certificate('/home/tihan/Desktop/new/follow.json')
firebase_admin.initialize_app(cred, {'databaseURL': 'https://drone-2037a-default-rtdb.firebaseio.com'})

# Setup MAVLink connection using DroneKit
connection_string = '/dev/ttyACM0'
baud_rate = 115200
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

# Function to calculate distance between two GPS coordinates
def get_distance_metres(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of Earth in meters
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance

# Function to calculate the bearing between two GPS coordinates
def get_bearing(lat1, lon1, lat2, lon2):
    dlon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dlon))
    initial_bearing = atan2(x, y)
    initial_bearing = initial_bearing * (180.0 / 3.14159265358979323846264338327950288)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

# Function to monitor leader's location and altitude from Firebase
def monitor_leader():
    leader_ref = db.reference('Leader_Location')
    comd_ref = db.reference('Leader_Control')
    takeoff_in_progress = False

    while True:
        try:
            leader_data = leader_ref.get()
            comd_data = comd_ref.get()
            cmd = comd_data.get('command')
            if cmd == 'RTL':
                update_flight_mode('RTL')
                break
            if leader_data:
                leader_lat = leader_data.get('latitude')
                leader_lon = leader_data.get('longitude')
                leader_alt = leader_data.get('altitude')

                if leader_alt >= 5 and not takeoff_in_progress:
                    print(f"Leader reached {leader_alt} meters, enabling takeoff for follower.")
                    takeoff(leader_alt)  # Follower takes off to the same altitude as the leader
                    takeoff_in_progress = True

                elif leader_alt < 5 and not takeoff_in_progress:
                    print("No command received in follower, leader has not reached 5 meters yet.")

                if takeoff_in_progress:
                    follow_leader(leader_lat, leader_lon, leader_alt, 5)  # Maintain 5 meters radial distance from leader

        except Exception as e:
            print(f"Error monitoring leader: {e}")
        
        time.sleep(2)  # Update GPS data every 2 seconds

# Function to takeoff follower to a specified altitude
def takeoff(target_altitude):
    if vehicle.mode.name != 'GUIDED':
        update_flight_mode('GUIDED')
    
    if vehicle.mode.name == 'GUIDED':
        if not vehicle.armed:
            vehicle.armed = True
            """while not vehicle.armed:
                print("Waiting for arming...")
                time.sleep(1)"""
        
        time.sleep(1)
        vehicle.simple_takeoff(target_altitude)
        print(f"Follower taking off to {target_altitude} meters...")

        while True:
            altitude = vehicle.location.global_relative_frame.alt
            print(f"Follower altitude: {altitude} meters")
            if altitude >= target_altitude * 0.95:  # Within 5% of target altitude
                print("Follower reached target altitude")
                break
            time.sleep(1)

# Function to follow the leader while maintaining a 5-meter radial distance
def follow_leader(leader_lat, leader_lon, leader_alt, target_distance):
    follower_location = vehicle.location.global_relative_frame
    follower_lat = follower_location.lat
    follower_lon = follower_location.lon

    distance_to_leader = get_distance_metres(follower_lat, follower_lon, leader_lat, leader_lon)
    bearing_to_leader = get_bearing(follower_lat, follower_lon, leader_lat, leader_lon)

    if distance_to_leader > target_distance:
        print(f"Follower is {distance_to_leader} meters away from leader, adjusting position.")
        # Calculate the new position to maintain the target distance from the leader
        target_lat = leader_lat - (target_distance / 111111) * cos(radians(bearing_to_leader))
        target_lon = leader_lon - (target_distance / (111111 * cos(radians(leader_lat)))) * sin(radians(bearing_to_leader))
        target_location = LocationGlobalRelative(target_lat, target_lon, leader_alt)
        vehicle.simple_goto(target_location)
    else:
        print(f"Follower is maintaining a distance of {distance_to_leader} meters from leader.")

# Function to update flight mode
def update_flight_mode(mode):
    try:
        if vehicle.mode.name != mode:
            vehicle.mode = VehicleMode(mode)
            while vehicle.mode.name != mode:
                print(f"Waiting for mode {mode}...")
                time.sleep(1)
        print(f"Flight mode set to: {mode}")
    except Exception as e:
        print(f"Failed to set mode: {e}")

def main():
    monitor_thread = threading.Thread(target=monitor_leader)
    monitor_thread.start()

    try:
        while True:
            time.sleep(1)  # Main thread does nothing, just waits
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        vehicle.close()  # Ensure the vehicle connection is closed properly

if __name__ == "__main__":
    main()
