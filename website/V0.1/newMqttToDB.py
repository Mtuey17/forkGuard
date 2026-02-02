import os
import time
from dotenv import load_dotenv

import paho.mqtt.client as paho
import mysql.connector
from mysql.connector import Error

load_dotenv()

# -------------------------
# CONFIG
# -------------------------
DB_CONFIG = {
    "host": os.getenv("DB_HOST", "localhost"),
    "user": os.getenv("DB_USER", "capstone_user"),
    "password": os.getenv("DB_PASSWORD", "LetsGetCoding!"),
    "database": os.getenv("DB_NAME", "WebsiteDemo"),
}

MQTT_HOST = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

BASE = "toyLift1"
TOPIC_SENSORS = f"{BASE}/sensors"
TOPIC_DRIVER = f"{BASE}/driver"  # payload = UUID

# These topics only publish when the event happens (increment by 1 each message)
EVENT_TOPICS = {
    f"{BASE}/gas": "gas",
    f"{BASE}/weightWarning": "weightWarning",
    f"{BASE}/weightError": "weightError",
    f"{BASE}/brakeWarning": "brakeWarning",
    f"{BASE}/brakeError": "brakeError",
}

CURRENT_DRIVER_UUID = None
CURRENT_DRIVER_NAME = None


# -------------------------
# DB HELPERS
# -------------------------
def get_conn():
    return mysql.connector.connect(**DB_CONFIG)


def ensure_driver_row(conn, driver_name: str):
    """
    Ensure a row exists for this driver in the drivers table.
    Requires columns:
      name, weightWarning, weightError, brakeWarning, brakeError, gas
    """
    # Best case: UNIQUE(name) exists, so ON DUPLICATE KEY works.
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO drivers
                  (name, weightWarning, weightError, brakeWarning, brakeError, gas)
                VALUES (%s, 0, 0, 0, 0, 0)
                ON DUPLICATE KEY UPDATE name = VALUES(name);
                """,
                (driver_name,),
            )
            conn.commit()
            return
    except Error:
        # If UNIQUE(name) does not exist, fall back to "check then insert"
        pass

    with conn.cursor() as cur:
        cur.execute("SELECT 1 FROM drivers WHERE name = %s LIMIT 1", (driver_name,))
        if cur.fetchone() is None:
            cur.execute(
                """
                INSERT INTO drivers
                  (name, weightWarning, weightError, brakeWarning, brakeError, gas)
                VALUES (%s, 0, 0, 0, 0, 0)
                """,
                (driver_name,),
            )
            conn.commit()


def increment_driver_counter(conn, driver_name: str, column: str):
    """
    Atomic increment: UPDATE drivers SET column = column + 1 WHERE name = ?
    """
    allowed = {"weightWarning", "weightError", "brakeWarning", "brakeError", "gas"}
    if column not in allowed:
        print(f"Invalid counter column: {column}")
        return

    ensure_driver_row(conn, driver_name)

    sql = f"UPDATE drivers SET {column} = {column} + 1 WHERE name = %s"
    with conn.cursor() as cur:
        cur.execute(sql, (driver_name,))
        conn.commit()
        if cur.rowcount != 1:
            print(f"Warning: increment affected {cur.rowcount} rows (driver={driver_name}, column={column})")


def resolve_driver_name(conn, uuid: str) -> str:
    with conn.cursor(dictionary=True) as cur:
        cur.execute(
            "SELECT name FROM drivers WHERE rfid_tag = %s LIMIT 1",
            (uuid,),
        )
        row = cur.fetchone()
        if row and row.get("name"):
            return row["name"]

    return f"Driver-{uuid[-6:]}" if uuid else "Unknown"



def add_sensor_data(conn, forklift_id: int, sensor_type: str, sensor_value: float):
    """
    Inserts sensor rows into sensor_data table.
    Assumes table exists: sensor_data(forklift_id, sensor_type, sensor_value)
    """
    sql = """
    INSERT INTO sensor_data (forklift_id, sensor_type, sensor_value)
    VALUES (%s, %s, %s)
    """
    with conn.cursor() as cur:
        cur.execute(sql, (forklift_id, sensor_type, sensor_value))
        conn.commit()


# -------------------------
# PAYLOAD PARSE
# -------------------------
def parse_kv_csv(payload: str) -> dict:
    """
    Parses: "MaxWeight:123, CurrentWeight:45"
    Returns dict with LOWERCASE keys.
    """
    out = {}
    for pair in payload.split(","):
        if ":" not in pair:
            continue
        key, value = pair.split(":", 1)
        out[key.strip().lower()] = value.strip()
    return out


# -------------------------
# MQTT CALLBACKS
# -------------------------
def on_connect(client, userdata, flags, rc):
    print("MQTT connected, rc =", rc)

    client.subscribe(TOPIC_SENSORS, qos=0)
    client.subscribe(TOPIC_DRIVER, qos=0)
    for t in EVENT_TOPICS:
        client.subscribe(t, qos=0)

    print("Subscribed to:")
    print(" -", TOPIC_SENSORS)
    print(" -", TOPIC_DRIVER)
    for t in EVENT_TOPICS:
        print(" -", t)


def on_message(client, userdata, message):
    global CURRENT_DRIVER_UUID, CURRENT_DRIVER_NAME

    topic = message.topic
    payload = message.payload.decode("utf-8", errors="replace").strip()

    # -------------------------
    # DRIVER UUID TOPIC
    # -------------------------
    if topic == TOPIC_DRIVER:
        CURRENT_DRIVER_UUID = payload

        try:
            conn = get_conn()
            CURRENT_DRIVER_NAME = resolve_driver_name(conn, payload)
            print(f"Active driver set: uuid={payload} -> name={CURRENT_DRIVER_NAME}")
        except Error as e:
            print("DB error resolving driver:", e)
            CURRENT_DRIVER_NAME = None
        finally:
            try:
                conn.close()
            except Exception:
                pass

        return

    # -------------------------
    # EVENT TOPICS (increment count)
    # -------------------------
    if topic in EVENT_TOPICS:
        # Recommended: ignore events until we know who the driver is
        if not CURRENT_DRIVER_NAME:
            print(f"Event {topic} received but no active driver set; ignoring")
            return

        column = EVENT_TOPICS[topic]

        try:
            conn = get_conn()
            increment_driver_counter(conn, CURRENT_DRIVER_NAME, column)
            print(f"Incremented {column} for {CURRENT_DRIVER_NAME}")
        except Error as e:
            print("DB error incrementing counter:", e)
        finally:
            try:
                conn.close()
            except Exception:
                pass

        return

    # -------------------------
    # SENSOR STREAM
    # -------------------------
    if topic == TOPIC_SENSORS:
        # Example payload: "MaxWeight:123, CurrentWeight:45"
        data = parse_kv_csv(payload)

        try:
            conn = get_conn()

            if "maxweight" in data:
                add_sensor_data(conn, forklift_id=1, sensor_type="max_weight", sensor_value=float(data["maxweight"]))

            if "currentweight" in data:
                add_sensor_data(
                    conn, forklift_id=1, sensor_type="current_weight", sensor_value=float(data["currentweight"])
                )

        except (Error, ValueError) as e:
            print("Sensor DB/parse error:", e)
        finally:
            try:
                conn.close()
            except Exception:
                pass


def on_disconnect(client, userdata, rc):
    print("MQTT disconnected, rc =", rc)


# -------------------------
# MAIN
# -------------------------
def main():
    client = paho.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    while True:
        try:
            print(f"Connecting to MQTT {MQTT_HOST}:{MQTT_PORT} ...")
            client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
            client.loop_forever()
        except Exception as e:
            print("MQTT error:", e)
            time.sleep(2)


if __name__ == "__main__":
    main()

