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
        "toggleCount",
        "calibrateZero",
        "calibrateTarget"
    ],
    "StreamDeck/IntakeRoller": [
        "intakeToggle",
        "outtakeToggle"
    ],
    "StreamDeck/Shooter": [
        "shooterToggle"
    ],
    "StreamDeck/Climb": [
        "climbUp",
        "climbDown"
    ],
    "limelight-back": [
        "piece_tx",
        "ta",
        "piece_distance",
        "has_target",
        "bbox",
        "hw"
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
            action = obj.get("action")
            table_name = obj.get("table")
            key = obj.get("key")

            # ===== PRESS — simula botão (True por 100ms, depois False) =====
            if action == "press" and table_name and key:
                nt_table = get_table(table_name)
                nt_table.putBoolean(key, True)
                await asyncio.sleep(0.1)
                nt_table.putBoolean(key, False)
                print(f"PRESS {table_name}/{key}")

            # ===== PUT GENÉRICO =====
            elif action == "put" and table_name and key:
                nt_table = get_table(table_name)
                value = obj.get("value")

                if isinstance(value, list):
                    nt_table.putNumberArray(key, value)
                elif isinstance(value, bool):
                    nt_table.putBoolean(key, value)
                elif isinstance(value, (int, float)):
                    nt_table.putNumber(key, value)
                else:
                    nt_table.putString(key, str(value))

                print(f"PUT {table_name}/{key} =", value)

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