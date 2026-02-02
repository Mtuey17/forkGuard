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
TOPIC_DRIVER = f"{BASE}/driver"  # payload = UUID (rfid_tag)

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


def normalize_uuid(u: str) -> str:
    """
    Normalize UUID/RFID tag to a consistent format.
    Even if you publish no spaces, this is harmless and prevents future bugs.
    """
    if not u:
        return ""
    return u.strip().replace(" ", "").upper()


def resolve_driver_name_by_uuid(conn, uuid: str) -> str | None:
    """
    Look up driver name in the drivers table using rfid_tag.
    Returns None if not found.
    """
    with conn.cursor(dictionary=True) as cur:
        cur.execute("SELECT name FROM drivers WHERE rfid_tag = %s LIMIT 1", (uuid,))
        row = cur.fetchone()
        if row and row.get("name"):
            return str(row["name"])
    return None


def ensure_driver_row_by_uuid(conn, uuid: str, name: str | None = None):
    """
    Ensure there is exactly ONE row per UUID (rfid_tag).
    This requires drivers.rfid_tag to be UNIQUE.

    If the row exists, keep counters as-is.
    If name is provided, update name (useful if you change a driver name later).
    """
    with conn.cursor() as cur:
        cur.execute(
            """
            INSERT INTO drivers (name, rfid_tag, weightWarning, weightError, brakeWarning, brakeError, gas)
            VALUES (%s, %s, 0, 0, 0, 0, 0)
            ON DUPLICATE KEY UPDATE
                name = COALESCE(VALUES(name), name);
            """,
            (name if name else None, uuid),
        )
        conn.commit()


def increment_counter_by_uuid(conn, uuid: str, column: str):
    """
    Increment counter for the driver identified by UUID (rfid_tag).
    This can only affect one row if rfid_tag is unique.
    """
    allowed = {"weightWarning", "weightError", "brakeWarning", "brakeError", "gas"}
    if column not in allowed:
        print(f"Invalid counter column: {column}")
        return

    sql = f"UPDATE drivers SET {column} = {column} + 1 WHERE rfid_tag = %s"
    with conn.cursor() as cur:
        cur.execute(sql, (uuid,))
        conn.commit()

        if cur.rowcount != 1:
            # rowcount 0 means UUID row missing (shouldn't happen if ensure is called)
            # rowcount >1 means your UNIQUE constraint is not actually unique
            print(f"Warning: increment affected {cur.rowcount} rows (uuid={uuid}, column={column})")


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
    payload = message.payload.decode("utf-8", errors="replace")

    # -------------------------
    # DRIVER UUID TOPIC
    # -------------------------
    if topic == TOPIC_DRIVER:
        uuid = normalize_uuid(payload)
        CURRENT_DRIVER_UUID = uuid

        try:
            conn = get_conn()

            # Try to resolve existing name for this UUID
            name = resolve_driver_name_by_uuid(conn, uuid)

            # If not found, pick a placeholder name (you can change later in DB)
            if not name:
                name = f"Driver-{uuid[-6:]}" if uuid else "Unknown"

            # Ensure one row exists for this UUID (never duplicates)
            ensure_driver_row_by_uuid(conn, uuid, name)

            CURRENT_DRIVER_NAME = name
            print(f"Active driver set: uuid={uuid} -> name={CURRENT_DRIVER_NAME}")

        except Error as e:
            print("DB error resolving/ensuring driver:", e)
            CURRENT_DRIVER_UUID = None
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
        if not CURRENT_DRIVER_UUID:
            print(f"Event {topic} received but no active driver UUID set; ignoring")
            return

        column = EVENT_TOPICS[topic]

        try:
            conn = get_conn()

            # Make sure row exists (safe even if it already exists)
            ensure_driver_row_by_uuid(conn, CURRENT_DRIVER_UUID, CURRENT_DRIVER_NAME)

            # Increment only the row matching this UUID
            increment_counter_by_uuid(conn, CURRENT_DRIVER_UUID, column)

            print(f"Incremented {column} for {CURRENT_DRIVER_NAME} ({CURRENT_DRIVER_UUID})")

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

