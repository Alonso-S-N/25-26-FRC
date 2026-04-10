const WebSocket = require("ws");

const CONFIG = {
    url: "ws://127.0.0.1:5810",
    reconnectInterval: 3000,
    maxReconnectAttempts: 10,
    pingInterval: 30000  // 30 segundos
};

let ws = null;
let reconnectAttempts = 0;
let isIntentionallyClosed = false;

function connect() {
    console.log(`🔗 Conectando a ${CONFIG.url}...`);
    ws = new WebSocket(CONFIG.url);

    ws.on("open", () => {
        console.log("✅ Conectado ao servidor WebSocket!");
        console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        console.log("📱 Comandos StreamDeck:");
        console.log("  i → Intake Toggle");
        console.log("  o → Outtake Toggle");
        console.log("  a → Intake Angle Toggle");
        console.log("  z → Calibrate Zero");
        console.log("  t → Calibrate Target");
        console.log("  q → Sair");
        console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        reconnectAttempts = 0;
    });

    ws.on("message", (data) => {
        try {
            const msg = JSON.parse(data);
            // Não loga tudo para não poluir console
            if (msg.topic && msg.value !== null) {
                console.log(`📡 ${msg.topic} = ${JSON.stringify(msg.value)}`);
            }
        } catch (e) {
            console.error("⚠️ Erro ao parsear JSON:", e.message);
        }
    });

    ws.on("close", () => {
        console.log("🔌 Desconectado do servidor");
        if (!isIntentionallyClosed) {
            attemptReconnect();
        }
    });

    ws.on("error", (error) => {
        console.error("⚠️ Erro WebSocket:", error.message);
    });

    ws.on("ping", () => {
        console.log("🏓 Ping recebido");
    });

    ws.on("pong", () => {
        console.log("🏓 Pong enviado");
    });
}

function attemptReconnect() {
    if (reconnectAttempts < CONFIG.maxReconnectAttempts) {
        reconnectAttempts++;
        const delay = CONFIG.reconnectInterval;
        console.log(`🔄 Reconectando em ${delay}ms (tentativa ${reconnectAttempts}/${CONFIG.maxReconnectAttempts})...`);
        setTimeout(connect, delay);
    } else {
        console.error("❌ Máximo de tentativas de reconexão atingido!");
        console.error("   Verifique se o servidor está rodando:");
        console.error("   python3 networktables_bridge.py --roborio 10.91.63.2 --port 5810");
    }
}

function sendPress(table, key) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        console.error(`❌ WebSocket não conectado! Não pude enviar ${table}/${key}`);
        return false;
    }

    try {
        const msg = JSON.stringify({
            action: "press",
            table: table,
            key: key
        });
        ws.send(msg);
        console.log(`📤 PRESS → ${table}/${key}`);
        return true;
    } catch (e) {
        console.error(`❌ Erro ao enviar: ${e.message}`);
        return false;
    }
}

function sendPut(table, key, value) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        console.error(`❌ WebSocket não conectado! Não pude enviar ${table}/${key}`);
        return false;
    }

    try {
        const msg = JSON.stringify({
            action: "put",
            table: table,
            key: key,
            value: value
        });
        ws.send(msg);
        console.log(`📤 PUT → ${table}/${key} = ${value}`);
        return true;
    } catch (e) {
        console.error(`❌ Erro ao enviar: ${e.message}`);
        return false;
    }
}

// ===== TECLADO =====
process.stdin.setRawMode(true);
process.stdin.resume();
process.stdin.setEncoding("utf8");

process.stdin.on("data", (key) => {
    if (key === "i") {
        sendPress("StreamDeck/IntakeRoller", "intakeToggle");
    } else if (key === "o") {
        sendPress("StreamDeck/IntakeRoller", "outtakeToggle");
    } else if (key === "a") {
        sendPress("StreamDeck/IntakeAngle", "toggleCount");
    } else if (key === "z") {
        sendPress("StreamDeck/IntakeAngle", "calibrateZero");
    } else if (key === "t") {
        sendPress("StreamDeck/IntakeAngle", "calibrateTarget");
    } else if (key === "u") {
        sendPress("StreamDeck/Climb", "climbUp",true);
    } else if (key === "d") {
        sendPress("StreamDeck/Climb", "climbDown",true);
    } else if (key === "q") {
        console.log("\n👋 Encerrando...");
        isIntentionallyClosed = true;
        if (ws) {
            ws.close();
        }
        process.exit(0);
    }
});

process.on("SIGINT", () => {
    console.log("\n👋 Encerrando (CTRL+C)...");
    isIntentionallyClosed = true;
    if (ws) {
        ws.close();
    }
    process.exit(0);
});
    
function sendPut(table, key, value) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        console.error(`❌ WebSocket não conectado!`);
        return false;
    }
    ws.send(JSON.stringify({ action: "put", table, key, value }));
    console.log(`📤 PUT → ${table}/${key} = ${value}`);
    return true;
}
// ===== INICIAR CONEXÃO =====
connect();