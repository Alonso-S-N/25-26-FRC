import asyncio
import json
import argparse
import time
from networktables import NetworkTables
import websockets

DEFAULT_ROBORIO = "10.91.63.2"
WS_HOST = "0.0.0.0"
DEFAULT_WS_PORT = 5810
WS_PATH = "/nt/dashboard"

POLL_INTERVAL = 0.15
clients = set()

# ================= STREAMDECK =================
SD_TABLE = "StreamDeck/IntakeAngle"
SD_KEY = "toggleCount"
toggle_count = 0
# ==============================================

TABLES_AND_KEYS = {
    "RobotStress": [
        "batteryVoltage",
        "totalCurrent",
        "drivetrainCurrent",
        "stressScore",
        "stressLevel",
        "speedScale",
        "chassisSpeed"
    ],

    "StreamDeck/IntakeAngle": [
        "toggleCount"
    ],

    "limelight-back": [
        "piece_tx",        # erro angular (graus)  
        "ta",              # área do alvo     
        "piece_distance",  # opcional
        "has_target",      # bool   
        "bbox",            # bounding box p/ UI    
        "hw"               # health/watchdog
    ],

    "limelight-front": [
        "tx",
        "tv",
        "ta",
        "hw"
    ],

    "Modes": [
        "AimLockLime4",
        "AimLockLime2",
        "AlignLime2"
    ]
}

def connect_nt(roborio_host):
    print(f"Inicializando NT -> {roborio_host}")
    NetworkTables.initialize(server=roborio_host)

    waited = 0.0
    while not NetworkTables.isConnected():
        time.sleep(0.1)
        waited += 0.1
        if waited > 10.0:
            print("Timeout NT")
            break

    if NetworkTables.isConnected():
        print("NT conectado")

def get_table(name):
    return NetworkTables.getTable(name)

def read_any(table, key):
    for fn in (
        table.getNumberArray,
        table.getNumber,
        table.getBoolean,
        table.getString
    ):
        try:
            v = fn(key, None)
            if v is not None:
                return list(v) if isinstance(v, (list, tuple)) else v
        except Exception:
            pass
    return None

async def poll_and_broadcast():
    last = {}

    while True:
        for table_name, keys in TABLES_AND_KEYS.items():
            table = get_table(table_name)
            for key in keys:
                topic = f"/{table_name}/{key}"
                val = read_any(table, key)

                if last.get(topic) != val:
                    last[topic] = val
                    msg = json.dumps({"topic": topic, "value": val})

                    dead = []
                    for ws in clients:
                        try:
                            await ws.send(msg)
                        except Exception:
                            dead.append(ws)

                    for ws in dead:
                        clients.discard(ws)

                    print(topic, val)

        await asyncio.sleep(POLL_INTERVAL)

async def handle_ws(ws):
    global toggle_count

    print("WS conectado:", ws.remote_address)
    clients.add(ws)

    try:
        # snapshot inicial
        for table_name, keys in TABLES_AND_KEYS.items():
            table = get_table(table_name)
            for key in keys:
                await ws.send(json.dumps({
                    "topic": f"/{table_name}/{key}",
                    "value": read_any(table, key)
                }))

        async for message in ws:
            obj = json.loads(message)

            # ===== STREAMDECK PRESS =====
            if obj.get("action") == "press":
                toggle_count += 1
                get_table(SD_TABLE).putNumber(SD_KEY, toggle_count)
                print(f"StreamDeck toggleCount = {toggle_count}")
                continue

            # ===== PUT GENÉRICO (dashboard) =====
            if obj.get("action") == "put":
                table = get_table(obj["table"])
                key = obj["key"]
                value = obj["value"]

                if isinstance(value, list):
                    table.putNumberArray(key, value)
                elif isinstance(value, bool):
                    table.putBoolean(key, value)
                elif isinstance(value, (int, float)):
                    table.putNumber(key, value)
                else:
                    table.putString(key, str(value))

                print("PUT:", obj)

    except Exception as e:
        print("WS erro:", e)
    finally:
        clients.discard(ws)
        print("WS desconectado")

async def main_async(roborio, port):
    connect_nt(roborio)

    server = await websockets.serve(handle_ws, WS_HOST, port)
    print(f"WS em ws://localhost:{port}{WS_PATH}")

    poll_task = asyncio.create_task(poll_and_broadcast())
    await server.wait_closed()
    await poll_task

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--roborio", default=DEFAULT_ROBORIO)
    parser.add_argument("--port", type=int, default=DEFAULT_WS_PORT)
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args.roborio, args.port))
    except KeyboardInterrupt:
        print("Encerrado")

if __name__ == "__main__":
    main()
