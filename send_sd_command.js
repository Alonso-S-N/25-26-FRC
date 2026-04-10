const WebSocket = require("ws");

const COMMAND = process.argv[2];

const COMMANDS = {
  // Intake Roller
  "INTAKE_ON":      { action: "press", table: "StreamDeck/IntakeRoller", key: "intakeToggle" },
  "INTAKE_OFF":     { action: "press", table: "StreamDeck/IntakeRoller", key: "outtakeToggle" },

  // Intake Angle
  "ANGLE_TOGGLE":   { action: "press", table: "StreamDeck/IntakeAngle",  key: "toggleCount" },
  "ANGLE_ZERO":     { action: "press", table: "StreamDeck/IntakeAngle",  key: "calibrateZero" },
  "ANGLE_TARGET":   { action: "press", table: "StreamDeck/IntakeAngle",  key: "calibrateTarget" },

  // Shooter
  "SHOOTER_ON":     { action: "press", table: "StreamDeck/Shooter",      key: "shooterToggle" },

  // Climb
  "CLIMB_UP":       { action: "press", table: "StreamDeck/Climb",        key: "climbUp" },
  "CLIMB_DOWN":     { action: "press", table: "StreamDeck/Climb",        key: "climbDown" },

  "CW":             { action: "put",   table: "StreamDeck/IntakeAngle",  key: "toggleCount", value: "CW" },
  "CCW":            { action: "put",   table: "StreamDeck/IntakeAngle",  key: "toggleCount", value: "CCW" },
  "STOP":           { action: "put",   table: "StreamDeck/IntakeAngle",  key: "toggleCount", value: "STOP" },
};

if (!COMMAND) {
  console.log("Uso: node send_sd_command.js <COMANDO>");
  console.log("\nComandos disponíveis:");
  Object.keys(COMMANDS).forEach(cmd => console.log(`  ${cmd}`));
  process.exit(1);
}

const payload = COMMANDS[COMMAND];

if (!payload) {
  console.error(`❌ Comando desconhecido: "${COMMAND}"`);
  console.log("Comandos válidos:", Object.keys(COMMANDS).join(", "));
  process.exit(1);
}

const ws = new WebSocket("ws://127.0.0.1:5810");

ws.on("open", () => {
  ws.send(JSON.stringify(payload));
  console.log(`📡 Enviado: ${COMMAND} →`, payload);
  setTimeout(() => ws.close(), 200);
});

ws.on("error", (err) => {
  console.error("❌ Erro ao conectar:", err.message);
  console.error("   Verifique se o bridge está rodando:");
  console.error("   python3 networktables_bridge.py --roborio 10.91.63.2 --port 5810");
  process.exit(1);
});