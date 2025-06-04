import time
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
import firebase_admin
from firebase_admin import credentials, db
from math import radians, sin, cos, sqrt, atan2, asin, degrees

# Initialize Firebase Admin SDK
cred = credentials.Certificate('/home/tihan/Desktop/leader /leader.json')
firebase_admin.initialize_app(cred, {'databaseURL': 'https://drone-2037a-default-rtdb.firebaseio.com'})

# Setup MAVLink connection using DroneKit
connection_string = '/dev/ttyACM0'
baud_rate = 115200
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

leader_airspeed = 4 # Define airspeed here

def update_location_to_firebase():
    ref = db.reference('Leader_Location')
    
    while True:
        try:
            current_location = vehicle.location.global_relative_frame
            latitude = current_location.lat
            longitude = current_location.lon
            altitude = current_location.alt

            # Update the Firebase database with the current location
            ref.update({
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude
            })

            print(f"Leader Location updated to Firebase: Lat {latitude}, Lon {longitude}, Alt {altitude}")
        except Exception as e:
            print(f"Error updating location to Firebase: {e}")

        time.sleep(1)  # Update every second

def update_flight_mode(mode):
    try:
        if vehicle.mode.name != mode:
            vehicle.mode = VehicleMode(mode)
            # Wait for mode to be confirmed
            while vehicle.mode.name != mode:
                print(f"Waiting for mode {mode}...")
                time.sleep(1)
        print(f"Flight mode set to: {mode}")
    except Exception as e:
        print(f"Failed to set mode: {e}")

def takeoff(target_altitude=5):
    if vehicle.mode.name != 'GUIDED':
        update_flight_mode('GUIDED')
    
    # Ensure the mode is properly set
    if vehicle.mode.name == 'GUIDED':
        # Arm the vehicle if not already armed
        if not vehicle.armed:
            vehicle.armed = True
            """while not vehicle.armed:
                print("Waiting for arming...")
                time.sleep(1)"""
        
        # Takeoff
        time.sleep(1)
        vehicle.simple_takeoff(target_altitude)
        print(f"Taking off to {target_altitude} meters...")

        # Wait until the vehicle reaches the target altitude
        while True:
            altitude = vehicle.location.global_relative_frame.alt
            print(f"Current altitude: {altitude} meters")
            if altitude >= target_altitude * 0.95:  # Within 5% of target altitude
                print("Reached target altitude")
                break
            time.sleep(1)
    else:
        print("Failed to set flight mode to GUIDED.")

def navigate_to_location(latitude, longitude):
    if vehicle.mode.name != 'GUIDED':
        update_flight_mode('GUIDED')
        
    # Set airspeed to achieve ground speed of approximately 2 meters per second
    vehicle.airspeed = leader_airspeed  # Use leader's airspeed
    print(f"Airspeed set to: {vehicle.airspeed} m/s")
    
    target_location = LocationGlobalRelative(latitude, longitude, vehicle.location.global_relative_frame.alt)
    print(f"Navigating to: Latitude {latitude}, Longitude {longitude}")
    vehicle.simple_goto(target_location)
    
    while True:
        current_location = vehicle.location.global_relative_frame
        distance_to_target = get_distance_metres(current_location.lat, current_location.lon, latitude, longitude)
        
        if distance_to_target < 5:  # Check if within 5 meters of the target
            print("Arrived at target location.")
            break
        time.sleep(1)

def get_distance_metres(lat1, lon1, lat2, lon2):
    from math import radians, sin, cos, sqrt, atan2
    R = 6371000  # Radius of Earth in meters

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)

    a = sin(dlat / 2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    
    return distance

def rtl():
    if vehicle.mode.name != 'RTL':
        update_flight_mode('RTL')
        print("Returning to Launch")

def monitor_firebase():
    ref = db.reference('Leader_Control')

    takeoff_in_progress = False
    
    while True:
        try:
            data = ref.get()
            if data:
                command = data.get('command')
                latitude = data.get('latitude')
                longitude = data.get('longitude')
                
                print(f"Command: {command}, Latitude: {latitude}, Longitude: {longitude}")

                if command:
                    if command == 'takeoff':
                        if not takeoff_in_progress:
                            takeoff(10)  # Take off to 5 meters
                            takeoff_in_progress = True
                    elif command == 'RTL':
                        rtl()
                        takeoff_in_progress = False  # Reset takeoff status for future commands

                if latitude and longitude:
                    navigate_to_location(float(latitude), float(longitude))
                    # Reset command and coordinates after processing
                    ref.update({'command': None, 'latitude': None, 'longitude': None})
        
        except Exception as e:
            print(f"Error in Firebase monitoring: {e}")
        
        time.sleep(1)  # Check Firebase every second

def main():
    # Start monitoring Firebase in a separate thread
    firebase_thread = threading.Thread(target=monitor_firebase)
    firebase_thread.start()
    location_thread = threading.Thread(target=update_location_to_firebase)
    location_thread.start()

    try:
        while True:
            time.sleep(1)  # Main thread does nothing, just waits
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        vehicle.close()  # Ensure the vehicle connection is closed properly

if __name__ == "__main__":
    main()
