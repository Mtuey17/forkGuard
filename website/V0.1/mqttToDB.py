import json
import paho.mqtt.client as paho
import os
from dotenv import load_dotenv
import mysql.connector
from mysql.connector import Error

load_dotenv()

DB_CONFIG = {
    'host': os.getenv('DB_HOST', 'localhost'),
    'user': os.getenv('DB_USER', 'capstone_user'),
    'password': os.getenv('DB_PASSWORD', 'LetsGetCoding!'),
    'database': os.getenv('DB_NAME', 'capstone'),
}

def get_conn():
    return mysql.connector.connect(**DB_CONFIG)

def add_sensor_data(forklift_id: int, sensor_type: str, sensor_value: float):

    conn = None
    try: 
        conn= get_conn()
        cursor = conn.cursor()
        sql = "INSERT INTO sensor_data (forklift_id, sensor_type, sensor_value) VALUES (%s, %s, %s)"
        cursor.execute(sql, (forklift_id, sensor_type, sensor_value))
        conn.commit()
        print("Inserted sensor_data id =", cursor.lastrowid)
        return cursor.lastrowid
    except Error as e:
        print("DB error (add_sensor_data):", e)
    finally:
        if conn:
            conn.close()

def start_session_by_uid(conn, uid, forklift_id=1):
    sql = """
    INSERT INTO sessions (user_id, forklift_id, start_time)
    SELECT u.user_id, %s, NOW()
    FROM users AS u
    WHERE u.rfid_tag = %s
      AND NOT EXISTS (
            SELECT 1 FROM sessions s
            WHERE s.user_id = u.user_id AND s.end_time IS NULL
      );
    """
    with conn.cursor() as cur:
        cur.execute(sql, (forklift_id, uid))
        conn.commit()
        if cur.rowcount == 1:
            return cur.lastrowid  # new session_id
        else:
            return None  # unknown UID or already active

def end_session_by_uid(conn, uid):
    sql = """
    UPDATE sessions s
    JOIN users u ON u.user_id = s.user_id
    SET s.end_time = NOW()
    WHERE u.rfid_tag = %s AND s.end_time IS NULL;
    """
    with conn.cursor() as cur:
        cur.execute(sql, (uid,))
        conn.commit()
        return cur.rowcount  # number of sessions closed
    
def uid_exists(conn, uid: str) -> bool:
    with conn.cursor() as cur:
        cur.execute("SELECT 1 FROM users WHERE rfid_tag = %s LIMIT 1", (uid,))
        return cur.fetchone() is not None

forkLiftID=0

def onMessage(client, userdata, message):
    sensorData = message.payload.decode("utf-8")
    print("Raw Sensor Data: ", sensorData)

    # Parse the sensor data into a dictionary (all keys always present)
    data_dict = {}
    try:
        for pair in sensorData.split(','):
            key, value = pair.split(':', 1)  # only split on first colon
            data_dict[key.strip().lower()] = value.strip()
        
        # Assign values to individual variables
        driver = str(data_dict['Driver'])
        maxWeight = float(data_dict['MaxWeight'])
        currentWeight  = float(data_dict['CurrentWeight'])
        '''roll   = float(data_dict['roll'])
        accel  = float(data_dict['accel'])
        switch = int(data_dict['switch'])
        uid    = str(data_dict['uid'])

        # Print the variables
        print(f"Weight: {weight}")
        print(f"Yaw: {yaw}")
        print(f"Pitch: {pitch}")
        print(f"Roll: {roll}")
        print(f"Acceleration: {accel}")
        print(f"Switch: {switch}")
        print(f"UID: {uid}")
'''
        print(f"max weight: {maxWeight}")

    except Exception as e:
        print("Error parsing sensor data:", e)
        return  # stop if the payload isn't in the expected format

    # Insert sensor rows
    add_sensor_data(1, "max weight",    maxWeight)
    add_sensor_data(1, "current weight",          currentWeight)
    '''add_sensor_data(1, "Pitch",        pitch)
    add_sensor_data(1, "Roll",         roll)
    add_sensor_data(1, "Acceleration", accel)
'''
    # Session toggle: only if UID exists. If not, ignore.
    conn = None
    try:
        conn = get_conn()
        uid="E3 18 8F 14"
        if not uid_exists(conn, uid):
            print(f"Unknown UID {uid}; skipping session toggle")
            return

        # Close if active; if nothing closed, start new
        closed = end_session_by_uid(conn, uid)
        if closed and closed > 0:
            print(f"Closed {closed} active session(s) for UID {uid}")
        else:
            sid = start_session_by_uid(conn, uid, forklift_id=1)
            if sid:
                print(f"Started session {sid} for UID {uid}")
            else:
                # Shouldn't normally happen now (since UID exists),
                # but NOT EXISTS guard could race—just log it.
                print(f"No session started for UID {uid} (already active?)")

    except Error as e:
        print("DB error (session toggle):", e)
    finally:
        if conn:
            conn.close()


client=paho.Client()
address="localhost"
topic="toylift1/sensors"
port=1883
client.connect(address,port)
client.subscribe(topic,qos=0)
client.on_message=onMessage
client.loop_forever()
