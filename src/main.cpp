#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_dsp.h"
#include "driver/i2s_std.h"

// ================= НАЛАШТУВАННЯ ПІНІВ =================
#define I2S_BCLK_PIN      GPIO_NUM_6
#define I2S_WS_PIN        GPIO_NUM_7
#define I2S_DIN_PIN       GPIO_NUM_4
#define RGB_LED_PIN       GPIO_NUM_8

#define RS232_TX_PIN      GPIO_NUM_16
#define RS232_RX_PIN      GPIO_NUM_17

// Вкажіть пін, якщо ваш RS485 модуль потребує керування напрямком (DE/RE). 
// Якщо модуль з авто-напрямком (напр. MAX13487), залиште -1
#define RS485_DE_PIN      -1 

// Піни для нових апаратних модулів
#define ESC_PWM_PIN       GPIO_NUM_2
#define ESC_SW_PIN        GPIO_NUM_3
#define CAM_RELAY_PIN     GPIO_NUM_5

#define FFT_SIZE          512
#define SAMPLE_RATE       16000
#define MAX_FFT_SIZE      1024
#define NUM_PATTERNS      6

// ================= СТРУКТУРА ПАТЕРНІВ =================
struct AcousticPattern {
    char name[20];
    int fMin;
    int fMax;
    float threshold;
    int minFrames;
    
    bool isMuted;       
    int baseLevel;      
    
    int frameCounter;
    bool isActive;
    unsigned long lastTriggerTime;
    float currentEnergy;
    
    int triggerCount;   
    int currentLevel;   
};

AcousticPattern patterns[NUM_PATTERNS] = {
    {"HUMAN_VOICE", 300,  2500, 0.03, 3,  false, 1, 0, false, 0, 0.0f, 0, 1}, 
    {"FPV_DRONE",   400,  1500, 0.05, 10, false, 2, 0, false, 0, 0.0f, 0, 2}, 
    {"CUSTOM_1",    1500, 3000, 0.10, 5,  false, 0, 0, false, 0, 0.0f, 0, 0},
    {"CUSTOM_2",    3000, 4500, 0.10, 5,  false, 0, 0, false, 0, 0.0f, 0, 0},
    {"CUSTOM_3",    4500, 6000, 0.10, 5,  false, 0, 0, false, 0, 0.0f, 0, 0},
    {"CUSTOM_4",    6000, 7500, 0.10, 5,  false, 0, 0, false, 0, 0.0f, 0, 0}
};

// ================= ГЛОБАЛЬНІ НАЛАШТУВАННЯ =================
bool agcEnabled = true;
float manualGain = 5.0f;
float dynamicGain = 5.0f; 
float agcMinGain = 2.0f; 
float agcMaxGain = 25.0f; 
bool masterMute = false;

// Налаштування Апаратних модулів
bool escModuleEnabled = false;
bool cameraPower = true;

// Стан мікрофона (захист від спаму)
bool isMicSleeping = false;
unsigned long micSleepUntil = 0;

// Стан ESC мотора
enum EscState { ESC_OFF, ESC_ARMING, ESC_READY };
EscState escState = ESC_OFF;
unsigned long escArmStartTime = 0;

int escTargetSpeed = 0;   // Цільова швидкість від користувача
int escCurrentSpeed = 0;  // Поточна швидкість мотора
unsigned long escPauseTime = 0;
bool escInPause = false;  // Прапорець затримки при зміні напрямку

Preferences prefs; 

// ================= ЗМІННІ ДЛЯ АВТО-НАВЧАННЯ =================
int recordingPatternId = -1;
unsigned long recordStartTime = 0;
int recordFrameCount = 0;
float accum_magnitude[FFT_SIZE / 2] = {0};
bool needConfigBroadcast = false;

// ================= ГЛОБАЛЬНІ МАСИВИ =================
__attribute__((aligned(16))) int32_t raw_pcm[FFT_SIZE * 2];
__attribute__((aligned(16))) float clean_samples[FFT_SIZE];
__attribute__((aligned(16))) float fft_input[FFT_SIZE * 2];
__attribute__((aligned(16))) float wind_func[FFT_SIZE];
__attribute__((aligned(16))) float magnitude[FFT_SIZE / 2];

// ================= СТАН СИСТЕМИ =================
enum SystemState { STATE_NORMAL, STATE_FAILSAFE, STATE_DEBUG };
SystemState currentState = STATE_NORMAL;

unsigned long lastRs232Ack = 0;
const unsigned long RS232_TIMEOUT = 3000;
bool firstCheckDone = false;
bool serversStarted = false;
bool globalAlarm = false; 

DNSServer dnsServer; 
WebServer server(80);
WebSocketsServer webSocket(81);
uint8_t activeWebClients = 0;

i2s_chan_handle_t rx_handle = NULL;

void setRGB(uint8_t r, uint8_t g, uint8_t b) { neopixelWrite(RGB_LED_PIN, r, g, b); }

// ================= ВІДПРАВКА В RS485 =================
void sendUart(String msg) {
    if (RS485_DE_PIN >= 0) digitalWrite(RS485_DE_PIN, HIGH);
    Serial1.println(msg);
    Serial1.flush(); // Чекаємо завершення передачі
    if (RS485_DE_PIN >= 0) digitalWrite(RS485_DE_PIN, LOW);
}

// ================= РОБОТА З ПАМ'ЯТТЮ (NVS) =================
void loadConfig() {
    prefs.begin("acoustic_cfg", false); 
    
    agcEnabled = prefs.getBool("agc", true);
    manualGain = prefs.getFloat("gain", 5.0f);
    agcMinGain = prefs.getFloat("agc_min", 2.0f);
    agcMaxGain = prefs.getFloat("agc_max", 25.0f);
    masterMute = prefs.getBool("m_mute", false);
    
    escModuleEnabled = prefs.getBool("esc_en", false);
    cameraPower = prefs.getBool("cam_pwr", true);
    
    for (int i = 0; i < NUM_PATTERNS; i++) {
        char key[16];
        sprintf(key, "p%d_fMin", i); patterns[i].fMin = prefs.getInt(key, patterns[i].fMin);
        sprintf(key, "p%d_fMax", i); patterns[i].fMax = prefs.getInt(key, patterns[i].fMax);
        sprintf(key, "p%d_thr", i);  patterns[i].threshold = prefs.getFloat(key, patterns[i].threshold);
        sprintf(key, "p%d_frm", i);  patterns[i].minFrames = prefs.getInt(key, patterns[i].minFrames);
        sprintf(key, "p%d_mute", i); patterns[i].isMuted = prefs.getBool(key, patterns[i].isMuted);
        sprintf(key, "p%d_lvl", i);  patterns[i].baseLevel = prefs.getInt(key, patterns[i].baseLevel);
        
        sprintf(key, "p%d_name", i);
        String savedName = prefs.getString(key, patterns[i].name);
        strlcpy(patterns[i].name, savedName.c_str(), sizeof(patterns[i].name));
        
        patterns[i].currentLevel = patterns[i].baseLevel;
        patterns[i].triggerCount = 0;
    }
}

void savePatternToNVS(int i) {
    char key[16];
    sprintf(key, "p%d_fMin", i); prefs.putInt(key, patterns[i].fMin);
    sprintf(key, "p%d_fMax", i); prefs.putInt(key, patterns[i].fMax);
    sprintf(key, "p%d_thr", i);  prefs.putFloat(key, patterns[i].threshold);
    sprintf(key, "p%d_frm", i);  prefs.putInt(key, patterns[i].minFrames);
    sprintf(key, "p%d_mute", i); prefs.putBool(key, patterns[i].isMuted);
    sprintf(key, "p%d_lvl", i);  prefs.putInt(key, patterns[i].baseLevel);
    sprintf(key, "p%d_name", i); prefs.putString(key, patterns[i].name);
}

void saveGlobalCfgToNVS() {
    prefs.putBool("agc", agcEnabled);
    prefs.putFloat("gain", manualGain);
    prefs.putFloat("agc_min", agcMinGain);
    prefs.putFloat("agc_max", agcMaxGain);
    prefs.putBool("m_mute", masterMute);
    prefs.putBool("esc_en", escModuleEnabled);
    prefs.putBool("cam_pwr", cameraPower);
}

void getConfigJsonString(String &out) {
    JsonDocument doc;
    doc["type"] = "init_data";
    doc["agc"] = agcEnabled;
    doc["gain"] = manualGain;
    doc["agc_min"] = agcMinGain;
    doc["agc_max"] = agcMaxGain;
    doc["master_mute"] = masterMute;
    doc["esc_enabled"] = escModuleEnabled;
    doc["cam_power"] = cameraPower;
    
    JsonArray arr = doc["patterns"].to<JsonArray>();
    for(int i=0; i<NUM_PATTERNS; i++) {
        JsonObject p = arr.add<JsonObject>();
        p["name"] = patterns[i].name;
        p["fMin"] = patterns[i].fMin;
        p["fMax"] = patterns[i].fMax;
        p["threshold"] = patterns[i].threshold;
        p["minFrames"] = patterns[i].minFrames;
        p["isMuted"] = patterns[i].isMuted;
        p["baseLevel"] = patterns[i].baseLevel;
    }
    serializeJson(doc, out);
}

// ================= ОБРОБКА СПІЛЬНИХ КОМАНД (WS + UART) =================
void processCommonCommand(JsonDocument& doc) {
    String type = doc["type"].as<String>();
    
    if (type == "set_pattern") {
        int id = doc["id"];
        if (id >= 0 && id < NUM_PATTERNS) {
            if (doc.containsKey("name")) strlcpy(patterns[id].name, doc["name"], sizeof(patterns[id].name));
            patterns[id].fMin = doc["fMin"];
            patterns[id].fMax = doc["fMax"];
            patterns[id].threshold = doc["thresh"];
            patterns[id].minFrames = doc["minFrames"];
            patterns[id].isMuted = doc["isMuted"];
            patterns[id].baseLevel = doc["baseLevel"];
            
            patterns[id].triggerCount = 0;
            patterns[id].currentLevel = patterns[id].baseLevel;
            savePatternToNVS(id);
        }
    }
    else if (type == "set_gain") {
        agcEnabled = doc["agc"];
        manualGain = doc["gain"];
        agcMinGain = doc["agc_min"];
        agcMaxGain = doc["agc_max"];
        if(!agcEnabled) dynamicGain = manualGain;
        saveGlobalCfgToNVS();
    }
    else if (type == "set_master_mute") {
        masterMute = doc["state"];
        saveGlobalCfgToNVS();
    }
    else if (type == "set_hw") {
        cameraPower = doc["cam"];
        escModuleEnabled = doc["esc"];
        ledcWrite(CAM_RELAY_PIN, cameraPower ? 410 : 205);
        saveGlobalCfgToNVS();
    }
    else if (type == "set_esc_speed") {
        escTargetSpeed = doc["val"];
    }
    else if (type == "start_rec") {
        int id = doc["id"];
        if (id >= 0 && id < NUM_PATTERNS) {
            recordingPatternId = id;
            recordStartTime = millis();
            recordFrameCount = 0;
            memset(accum_magnitude, 0, sizeof(accum_magnitude));
        }
    }
}

// ================= ОСНОВНИЙ WEB UI =================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="uk">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-C6 Multi-Pattern Node</title>
    <style>
        :root { --bg:#121214; --card:#1e1e24; --accent:#00e676; --alarm:#ff1744; --text:#e0e0e0; }
        body { font-family:system-ui, sans-serif; background:var(--bg); color:var(--text); margin:0; padding:15px; }
        .grid { display:grid; grid-template-columns:repeat(auto-fit, minmax(320px, 1fr)); gap:15px; }
        .card { background:var(--card); padding:15px; border-radius:10px; border:1px solid #2a2a35; }
        h2 { margin-top:0; font-size:1.1rem; color:#8b8ba7; text-transform:uppercase; margin-bottom: 15px; display:flex; justify-content:space-between; align-items:center;}
        .status-badge { padding:6px 12px; border-radius:20px; font-weight:bold; font-size:0.85rem; }
        .status-ok { background:rgba(0,230,118,0.15); color:var(--accent); }
        .status-alarm { background:rgba(255,23,68,0.2); color:var(--alarm); animation:pulse 1s infinite; }
        @keyframes pulse { 0%,100%{opacity:1;} 50%{opacity:0.5;} }
        canvas { width:100%; height:200px; background:#0a0a0c; border-radius:6px; }
        
        .range-container { position:relative; width:100%; height:30px; margin-top:8px; }
        .range-slider { position:absolute; width:100%; top:0; pointer-events:none; -webkit-appearance:none; background:transparent; margin:0; }
        .range-slider::-webkit-slider-thumb { pointer-events:auto; -webkit-appearance:none; width:18px; height:18px; border-radius:50%; background:var(--accent); cursor:pointer; position:relative; z-index:2; margin-top:-7px;}
        .range-slider:disabled::-webkit-slider-thumb { background:#8b8ba7; }
        .range-slider::-webkit-slider-runnable-track { -webkit-appearance:none; background:transparent; border:none; }
        .range-track { position:absolute; top:10px; height:4px; background:#3d3d4e; width:100%; border-radius:2px; z-index:0; }
        .range-fill { position:absolute; top:10px; height:4px; background:var(--accent); z-index:1; border-radius:2px; }
        
        .pattern-row { background:#2a2a35; padding:12px; border-radius:8px; margin-bottom:12px; border-left: 4px solid #3d3d4e;}
        .pattern-row.active { border-left-color: var(--alarm); background: #352528; }
        .pattern-row.muted { opacity: 0.6; border-left-color: #555;}
        
        .pattern-header { display:flex; flex-wrap:wrap; justify-content:space-between; align-items:center; margin-bottom:10px; gap:10px;}
        .btn-rec { background:#ff1744; color:#fff; border:none; padding:4px 10px; border-radius:4px; font-weight:bold; cursor:pointer; font-size:0.75rem; transition:0.2s; }
        .btn-rec:active { transform: scale(0.95); }
        
        .btn-link { background:#2979ff; color:#fff; text-decoration:none; padding:6px 12px; border-radius:6px; font-size:0.8rem; font-weight:bold; }
        .btn-master-mute { background:#ff1744; color:#fff; border:none; padding:6px 12px; border-radius:6px; font-weight:bold; cursor:pointer; font-size:0.9rem;}
        .btn-master-mute.muted { background:#8b8ba7; color:#121214; }
        
        .pat-inputs { display:flex; flex-wrap:wrap; gap:8px; }
        .pat-inputs > div { flex:1; min-width:80px; }
        .pat-inputs input[type="number"] { width:100%; background:#1e1e24; color:#fff; border:1px solid #3d3d4e; padding:6px; border-radius:4px; box-sizing:border-box; font-family:monospace;}
        .pat-name-input { background:transparent; color:var(--accent); border:1px solid #3d3d4e; border-radius:4px; padding:4px 6px; font-weight:bold; width:130px; font-size:0.9rem;}
        
        .log-box { height:150px; overflow-y:auto; background:#0a0a0c; padding:8px; border-radius:6px; font-family:monospace; font-size:0.8rem; }
        
        /* ESC Buttons */
        .btn-esc { flex:1; min-width:40px; padding:10px 2px; border:none; border-radius:6px; font-weight:bold; cursor:pointer; font-size:0.75rem; transition:0.2s;}
        .btn-esc:active { transform: scale(0.95); }
        .btn-esc-rev { background:rgba(255,23,68,0.2); color:var(--alarm); border:1px solid rgba(255,23,68,0.4);}
        .btn-esc-stop { background:#3d3d4e; color:#fff; border:1px solid #555; font-size:0.85rem;}
        .btn-esc-fwd { background:rgba(0,230,118,0.2); color:var(--accent); border:1px solid rgba(0,230,118,0.4);}
    </style>
</head>
<body>
    <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:15px; flex-wrap:wrap; gap:10px;">
        <h1 style="font-size:1.3rem; margin:0;">Вузол Аналізу (6 Патернів)</h1>
        <div style="display:flex; gap:10px; align-items:center;">
            <a href="/config" class="btn-link">⚙️ Файли Налаштувань</a>
            <div id="sysState" class="status-badge status-ok">CONNECTING...</div>
        </div>
    </div>

    <div class="grid">
        <div class="card" style="grid-column: 1 / -1;">
            <h2>
                Спектр FFT (0 - 8000 Гц)
                <button id="masterMuteBtn" class="btn-master-mute" onclick="toggleMasterMute()">🔊 MASTER MUTE: OFF</button>
            </h2>
            <div style="position:relative;">
                <canvas id="fftCanvas"></canvas>
            </div>
            
            <div style="margin-top:10px;">
                <div style="display:flex; justify-content:space-between; font-size:0.85rem; margin-bottom:4px;">
                    <span>VU Meter (Амплітуда)</span>
                    <span id="rs232State" style="color:#8b8ba7;">RS485: UNKNOWN</span>
                </div>
                <div style="background:#0a0a0c; height:12px; border-radius:6px; overflow:hidden;">
                    <div id="vuBar" style="width:0%; height:100%; background:var(--accent); transition: width 0.08s linear;"></div>
                </div>
            </div>
            
            <div style="margin-top:20px; background:#2a2a35; padding:15px; border-radius:8px;">
                <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom: 15px;">
                    <label style="display:flex; align-items:center; gap:8px; font-size:1rem; cursor:pointer;">
                        <input type="checkbox" id="agcEnable" onchange="sendGain()" style="width:18px; height:18px;">
                        <b>AGC (Авто-Чутливість)</b>
                    </label>
                    <span style="color:var(--accent); font-weight:bold; font-size:0.9rem;">Поточний: <span id="dynGainVal">1.0</span>x</span>
                </div>
                
                <div style="margin-bottom:20px;">
                    <div style="display:flex; justify-content:space-between; font-size:0.85rem; color:#8b8ba7; margin-bottom:5px;">
                        <span>Ліміти Авто-AGC:</span>
                        <span style="color:#fff;"><span id="agcMinVal">1.0</span>x - <span id="agcMaxVal">40.0</span>x</span>
                    </div>
                    <div class="range-container">
                        <div class="range-track"></div>
                        <div class="range-fill" id="agcRangeFill" style="background:#00e676;"></div>
                        <input type="range" class="range-slider" id="agcMinSlider" min="1.0" max="40.0" step="0.5" value="1.0" oninput="updateAgcRangeUI()" onchange="sendGain()">
                        <input type="range" class="range-slider" id="agcMaxSlider" min="1.0" max="40.0" step="0.5" value="40.0" oninput="updateAgcRangeUI()" onchange="sendGain()">
                    </div>
                </div>
                <div>
                    <div style="display:flex; justify-content:space-between; font-size:0.85rem; color:#8b8ba7; margin-bottom:5px;">
                        <span>Ручний Gain:</span>
                        <span style="color:#fff;"><span id="manualGainVal">5.0</span>x</span>
                    </div>
                    <input type="range" id="manualGainSlider" min="1.0" max="40.0" step="0.5" value="5.0" oninput="updateManualGainUI()" onchange="sendGain()" style="width:100%;">
                </div>
            </div>
        </div>

        <div class="card" style="grid-column: 1 / -1;">
            <h2>🔌 Апаратні Модулі</h2>
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:15px; flex-wrap:wrap; gap:10px;">
                <span style="font-weight:bold;">Живлення Камери (РЕЛЕ)</span>
                <button id="camPwrBtn" onclick="toggleCamPwr()" style="border:none; padding:8px 16px; border-radius:6px; font-weight:bold; cursor:pointer;">ЗАВАНТАЖЕННЯ</button>
            </div>
            
            <hr style="border:0; border-top:1px solid #3d3d4e; margin:15px 0;">
            
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:15px;">
                <label style="display:flex; align-items:center; gap:8px; cursor:pointer; font-weight:bold;">
                    <input type="checkbox" id="escEnable" onchange="toggleEscMod()" style="width:18px; height:18px;">
                    Модуль ESC (Колектор)
                </label>
                <span id="escStatus" style="color:#8b8ba7; font-weight:bold; font-size:0.9rem;">ОФФЛАЙН</span>
            </div>
            
            <div id="escControlBox" style="opacity:0.5; pointer-events:none; background:#0a0a0c; padding:15px; border-radius:8px;">
                <div style="display:flex; justify-content:space-between; margin-bottom:12px; font-size:0.85rem; color:#8b8ba7;">
                    <span>Керування напрямком</span>
                    <span style="color:#fff; font-weight:bold;">Поточна: <span id="escSpeedVal" style="color:#8b8ba7;">STOP</span></span>
                </div>
                <div style="display:flex; gap:6px; flex-wrap:wrap;">
                    <button class="btn-esc btn-esc-rev" onclick="setEscSpeed(-100)">MAX R</button>
                    <button class="btn-esc btn-esc-rev" onclick="setEscSpeed(-80)">MID R</button>
                    <button class="btn-esc btn-esc-rev" onclick="setEscSpeed(-60)">MIN R</button>
                    <button class="btn-esc btn-esc-stop" onclick="setEscSpeed(0)">STOP</button>
                    <button class="btn-esc btn-esc-fwd" onclick="setEscSpeed(60)">MIN F</button>
                    <button class="btn-esc btn-esc-fwd" onclick="setEscSpeed(80)">MID F</button>
                    <button class="btn-esc btn-esc-fwd" onclick="setEscSpeed(100)">MAX F</button>
                </div>
            </div>
        </div>

        <div class="card" style="grid-column: 1 / -1;">
            <h2>Панель Патернів</h2>
            <div id="patternsContainer">
                <div style="text-align:center; padding: 20px; color:#8b8ba7;">Завантаження конфігурації...</div>
            </div>
        </div>

        <div class="card" style="grid-column: 1 / -1;">
            <h2>Лог подій (Алярми)</h2>
            <div class="log-box" id="logBox"></div>
        </div>
    </div>

<script>
    let ws = new WebSocket(`ws://${location.hostname}:81`);
    const canvas = document.getElementById('fftCanvas');
    const ctx = canvas.getContext('2d');
    let patternsData = [];
    let activeStates = [false, false, false, false, false, false];
    let isMasterMute = false;
    let camIsOn = true;
    const VISUAL_BOOST = 25.0; 

    ws.onopen = () => { ws.send(JSON.stringify({type: "get_config"})); };
    ws.onmessage = (event) => {
        try {
            let msg = JSON.parse(event.data);
            if (msg.type === 'fft') {
                activeStates = msg.active_patterns;
                
                msg.triggers.forEach((cnt, idx) => {
                    let cEl = document.getElementById(`trigCnt-${idx}`);
                    let lEl = document.getElementById(`curLvl-${idx}`);
                    if (cEl) cEl.innerText = cnt;
                    if (lEl) {
                        lEl.innerText = msg.levels[idx];
                        lEl.style.color = msg.levels[idx] >= 4 ? "#ff1744" : "#e0e0e0";
                    }
                });
                
                requestAnimationFrame(() => {
                    drawFFT(msg.data, msg.vu);
                    updateUIState(msg.state, msg.rs232);
                    updatePatternRows();
                    
                    if(msg.cam_pwr !== undefined) {
                        camIsOn = msg.cam_pwr;
                        syncCamBtn();
                    }
                    
                    let escStat = document.getElementById('escStatus');
                    let escBox = document.getElementById('escControlBox');
                    if (!document.getElementById('escEnable').checked) {
                        escStat.innerText = "ВИМКНЕНО";
                        escStat.style.color = "#8b8ba7";
                        escBox.style.opacity = 0.5;
                        escBox.style.pointerEvents = 'none';
                    } else {
                        escBox.style.opacity = 1;
                        escBox.style.pointerEvents = 'auto';
                        if (msg.esc_state === 0) { escStat.innerText = "ВИМИКАЄТЬСЯ..."; escStat.style.color = "#8b8ba7"; }
                        else if (msg.esc_state === 1) { escStat.innerText = "⏳ ІНІЦІАЛІЗАЦІЯ..."; escStat.style.color = "#ffb300"; }
                        else if (msg.esc_state === 2) { escStat.innerText = "✅ ГОТОВО"; escStat.style.color = "#00e676"; }
                    }

                    let eSpeed = msg.esc_target;
                    let valEl = document.getElementById('escSpeedVal');
                    
                    if (msg.esc_in_pause) {
                        valEl.innerText = "⏳ PAUSE...";
                        valEl.style.color = "#ffb300";
                    } else {
                        let spLabel = "STOP";
                        if (eSpeed === -100) spLabel = "MAX R";
                        else if (eSpeed === -80) spLabel = "MID R";
                        else if (eSpeed === -60) spLabel = "MIN R";
                        else if (eSpeed === 0) spLabel = "STOP";
                        else if (eSpeed === 60) spLabel = "MIN F";
                        else if (eSpeed === 80) spLabel = "MID F";
                        else if (eSpeed === 100) spLabel = "MAX F";
                        else spLabel = eSpeed + "%";
                        
                        valEl.innerText = spLabel;
                        if(eSpeed > 0) valEl.style.color = "var(--accent)";
                        else if (eSpeed < 0) valEl.style.color = "var(--alarm)";
                        else valEl.style.color = "#8b8ba7";
                    }

                    msg.energies.forEach((en, idx) => {
                        let el = document.getElementById(`energyVal-${idx}`);
                        if(el) el.innerText = en.toFixed(3);
                    });
                    if(document.getElementById('agcEnable').checked) {
                        document.getElementById('dynGainVal').innerText = msg.dyn_gain.toFixed(1);
                    }
                });
            } else if (msg.type === 'sleep_status') {
                ctx.fillStyle = 'rgba(10, 10, 12, 0.85)';
                ctx.fillRect(0, 0, canvas.width, canvas.height);
                ctx.fillStyle = '#ff1744';
                ctx.font = 'bold 18px sans-serif';
                ctx.textAlign = 'center';
                ctx.fillText(`💤 ЗАХИСТ ВІД СПАМУ! Охолодження: ${msg.left} сек`, canvas.width/2, canvas.height/2);
            } else if (msg.type === 'alarm_event') {
                addLog(`⚠️ [${new Date().toLocaleTimeString()}] АЛЯРМ: ${msg.name} (Рівень ${msg.lvl})`);
            } else if (msg.type === 'init_data') {
                patternsData = msg.patterns;
                isMasterMute = msg.master_mute;
                syncMasterMuteBtn();
                
                document.getElementById('escEnable').checked = msg.esc_enabled;
                camIsOn = msg.cam_power;
                syncCamBtn();

                document.getElementById('agcEnable').checked = msg.agc;
                document.getElementById('manualGainSlider').value = msg.gain;
                document.getElementById('manualGainVal').innerText = msg.gain.toFixed(1);
                document.getElementById('agcMinSlider').value = msg.agc_min;
                document.getElementById('agcMaxSlider').value = msg.agc_max;
                updateAgcRangeUI();
                syncGainControls();
                renderPatternsUI(); 
            }
        } catch (e) {}
    };

    function toggleCamPwr() {
        camIsOn = !camIsOn;
        ws.send(JSON.stringify({type: 'set_hw', cam: camIsOn, esc: document.getElementById('escEnable').checked}));
        syncCamBtn();
    }

    function syncCamBtn() {
        let btn = document.getElementById('camPwrBtn');
        if (camIsOn) {
            btn.innerText = "✅ УВІМКНЕНО";
            btn.style.background = "#00e676";
            btn.style.color = "#121214";
        } else {
            btn.innerText = "❌ ВИМКНЕНО";
            btn.style.background = "#3d3d4e";
            btn.style.color = "#fff";
        }
    }

    function toggleEscMod() {
        let en = document.getElementById('escEnable').checked;
        ws.send(JSON.stringify({type: 'set_hw', cam: camIsOn, esc: en}));
        if (!en) {
            setEscSpeed(0);
        }
    }

    function setEscSpeed(val) {
        ws.send(JSON.stringify({type: 'set_esc_speed', val: val}));
    }

    function renderPatternsUI() {
        const container = document.getElementById('patternsContainer');
        let htmlStr = ''; 
        patternsData.forEach((p, idx) => {
            let muteClass = p.isMuted ? 'muted' : '';
            htmlStr += `
            <div class="pattern-row ${muteClass}" id="patRow-${idx}">
                <div class="pattern-header">
                    <div style="display:flex; align-items:center; gap:8px;">
                        <span style="color:#8b8ba7;">[${idx}]</span>
                        <input type="text" class="pat-name-input" id="name-${idx}" value="${p.name}" onchange="savePat(${idx})">
                        <label style="cursor:pointer; display:flex; align-items:center; gap:4px; font-size:0.8rem; margin-left:5px;">
                            <input type="checkbox" id="mute-${idx}" ${p.isMuted ? 'checked' : ''} onchange="savePat(${idx})"> Mute
                        </label>
                    </div>
                    <div style="display:flex; gap:10px; align-items:center;">
                        <select id="lvl-${idx}" onchange="savePat(${idx})" style="background:#1e1e24; color:#fff; border:1px solid #3d3d4e; border-radius:4px; padding:4px; font-size:0.8rem;">
                            <option value="0" ${p.baseLevel==0?'selected':''}>0 - Інфо</option>
                            <option value="1" ${p.baseLevel==1?'selected':''}>1 - Увага</option>
                            <option value="2" ${p.baseLevel==2?'selected':''}>2 - Тривога</option>
                            <option value="3" ${p.baseLevel==3?'selected':''}>3 - Критично</option>
                        </select>
                        <button id="recBtn-${idx}" class="btn-rec" onclick="startRec(${idx})">🔴 REC</button>
                    </div>
                </div>
                
                <div style="display:flex; justify-content:space-between; font-size:0.75rem; color:#8b8ba7; margin-bottom:10px; background:#1a1a20; padding:6px 10px; border-radius:4px;">
                    <span>Спрацювань ескалації: <b id="trigCnt-${idx}" style="color:#fff;">0</b></span>
                    <span>Поточний рівень: <b id="curLvl-${idx}" style="color:#fff;">${p.baseLevel}</b> / 5</span>
                </div>

                <div style="display:flex; justify-content:space-between; font-size:0.8rem; color:#8b8ba7; margin-bottom:5px;">
                    <span>Смуга (<span id="fMinVal-${idx}">${p.fMin}</span> - <span id="fMaxVal-${idx}">${p.fMax}</span> Гц)</span>
                    <span id="patStatus-${idx}" style="width:45px; text-align:right;">IDLE</span>
                </div>
                
                <div class="range-container">
                    <div class="range-track"></div>
                    <div class="range-fill" id="rangeFill-${idx}"></div>
                    <input type="range" class="range-slider" id="fMinSlider-${idx}" min="0" max="8000" step="50" value="${p.fMin}" oninput="updateRangeUI(${idx})" onchange="savePat(${idx})">
                    <input type="range" class="range-slider" id="fMaxSlider-${idx}" min="0" max="8000" step="50" value="${p.fMax}" oninput="updateRangeUI(${idx})" onchange="savePat(${idx})">
                </div>
                <div class="pat-inputs" style="margin-top:15px;">
                    <div>
                        <small style="display:flex; justify-content:space-between;">Поріг <span id="energyVal-${idx}" style="color:var(--accent);">0.000</span></small>
                        <input type="number" step="0.01" id="thresh-${idx}" value="${p.threshold.toFixed(2)}" onchange="savePat(${idx})">
                    </div>
                    <div>
                        <small>Min Frames</small>
                        <input type="number" id="frames-${idx}" value="${p.minFrames}" onchange="savePat(${idx})">
                    </div>
                </div>
            </div>`;
        });
        container.innerHTML = htmlStr;
        patternsData.forEach((p, idx) => { updateRangeUI(idx); }); 
    }

    function toggleMasterMute() {
        isMasterMute = !isMasterMute;
        ws.send(JSON.stringify({type: 'set_master_mute', state: isMasterMute}));
        syncMasterMuteBtn();
    }
    
    function syncMasterMuteBtn() {
        let btn = document.getElementById('masterMuteBtn');
        if(isMasterMute) {
            btn.innerText = "🔇 MASTER MUTE: ON";
            btn.className = "btn-master-mute muted";
        } else {
            btn.innerText = "🔊 MASTER MUTE: OFF";
            btn.className = "btn-master-mute";
        }
    }

    function startRec(idx) {
        let btn = document.getElementById(`recBtn-${idx}`);
        btn.innerText = '⏳ СЛУХАЮ...';
        btn.disabled = true;
        ws.send(JSON.stringify({type: 'start_rec', id: idx}));
    }

    function updateRangeUI(idx) {
        let minEl = document.getElementById(`fMinSlider-${idx}`);
        let maxEl = document.getElementById(`fMaxSlider-${idx}`);
        let minVal = parseInt(minEl.value), maxVal = parseInt(maxEl.value);
        if (minVal >= maxVal) { let tmp = minEl.value; minEl.value = maxEl.value; maxEl.value = tmp; minVal = parseInt(minEl.value); maxVal = parseInt(maxEl.value); }
        document.getElementById(`fMinVal-${idx}`).innerText = minVal;
        document.getElementById(`fMaxVal-${idx}`).innerText = maxVal;
        let fill = document.getElementById(`rangeFill-${idx}`);
        fill.style.left = (minVal / 8000) * 100 + '%';
        fill.style.width = ((maxVal - minVal) / 8000) * 100 + '%';
    }

    function savePat(idx) {
        updateRangeUI(idx);
        let nameVal = document.getElementById(`name-${idx}`).value;
        let isMuted = document.getElementById(`mute-${idx}`).checked;
        let baseLevel = parseInt(document.getElementById(`lvl-${idx}`).value);
        
        let row = document.getElementById(`patRow-${idx}`);
        if(isMuted) row.classList.add('muted'); else row.classList.remove('muted');

        ws.send(JSON.stringify({
            type: 'set_pattern', id: idx,
            name: nameVal,
            fMin: parseInt(document.getElementById(`fMinSlider-${idx}`).value),
            fMax: parseInt(document.getElementById(`fMaxSlider-${idx}`).value),
            thresh: parseFloat(document.getElementById(`thresh-${idx}`).value),
            minFrames: parseInt(document.getElementById(`frames-${idx}`).value),
            isMuted: isMuted,
            baseLevel: baseLevel
        }));
    }

    function updateAgcRangeUI() {
        let minEl = document.getElementById('agcMinSlider'), maxEl = document.getElementById('agcMaxSlider');
        let minVal = parseFloat(minEl.value), maxVal = parseFloat(maxEl.value);
        if (minVal >= maxVal) { let tmp = minEl.value; minEl.value = maxEl.value; maxEl.value = tmp; minVal = parseFloat(minEl.value); maxVal = parseFloat(maxEl.value); }
        document.getElementById('agcMinVal').innerText = minVal.toFixed(1);
        document.getElementById('agcMaxVal').innerText = maxVal.toFixed(1);
        let fill = document.getElementById('agcRangeFill');
        fill.style.left = ((minVal - 1.0) / 39.0) * 100 + '%';
        fill.style.width = ((maxVal - minVal) / 39.0) * 100 + '%';
    }

    function updateManualGainUI() { document.getElementById('manualGainVal').innerText = parseFloat(document.getElementById('manualGainSlider').value).toFixed(1); }
    function syncGainControls() {
        let isAgc = document.getElementById('agcEnable').checked;
        document.getElementById('manualGainSlider').disabled = isAgc;
        document.getElementById('agcMinSlider').disabled = !isAgc;
        document.getElementById('agcMaxSlider').disabled = !isAgc;
    }

    function sendGain() {
        syncGainControls();
        updateAgcRangeUI();
        updateManualGainUI();
        ws.send(JSON.stringify({
            type: 'set_gain', agc: document.getElementById('agcEnable').checked,
            gain: parseFloat(document.getElementById('manualGainSlider').value),
            agc_min: parseFloat(document.getElementById('agcMinSlider').value),
            agc_max: parseFloat(document.getElementById('agcMaxSlider').value)
        }));
    }

    function updatePatternRows() {
        activeStates.forEach((isActive, idx) => {
            let row = document.getElementById(`patRow-${idx}`), stat = document.getElementById(`patStatus-${idx}`);
            if(row && stat) {
                if(isActive) { row.classList.add('active'); stat.innerText = 'ТРИВОГА'; stat.style.color = 'var(--alarm)'; }
                else { row.classList.remove('active'); stat.innerText = 'IDLE'; stat.style.color = '#8b8ba7'; }
            }
        });
    }

    function drawFFT(bins, vu) {
        if (!canvas.width || !canvas.height) { canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; }
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        const barWidth = canvas.width / bins.length;
        patternsData.forEach((p, idx) => {
            let startX = (p.fMin / 31.25) * barWidth, width = ((p.fMax - p.fMin) / 31.25) * barWidth;
            ctx.fillStyle = activeStates[idx] ? 'rgba(255, 23, 68, 0.25)' : 'rgba(0, 230, 118, 0.08)';
            ctx.fillRect(startX, 0, width, canvas.height);
        });
        ctx.fillStyle = '#8b8ba7';
        for (let i = 0; i < bins.length; i++) {
            let barHeight = Math.pow(bins[i] * VISUAL_BOOST, 0.85) * canvas.height;
            if (barHeight > canvas.height) barHeight = canvas.height;
            ctx.fillRect(i * barWidth, canvas.height - barHeight, barWidth - 1, barHeight);
        }
        document.getElementById('vuBar').style.width = Math.min(100, vu * 100 * (VISUAL_BOOST / 4)) + '%';
    }

    function updateUIState(state, rs232) {
        const badge = document.getElementById('sysState');
        badge.innerText = state;
        badge.className = (state === 'FAILSAFE') ? 'status-badge status-alarm' : 'status-badge status-ok';
        document.getElementById('rs232State').innerText = rs232 ? 'RS485: OK' : 'RS485: TIMEOUT';
    }

    function addLog(text) {
        const box = document.getElementById('logBox');
        box.innerHTML += `<div>${text}</div>`;
        box.scrollTop = box.scrollHeight;
    }
</script>
</body>
</html>
)rawliteral";

// ================= СТОРІНКА МАСОВОГО ІМПОРТУ/ЕКСПОРТУ ФАЙЛІВ (/config) =================
const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="uk">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Робота з файлами конфігурації</title>
    <style>
        :root { --bg:#121214; --card:#1e1e24; --accent:#00e676; --text:#e0e0e0; }
        body { font-family:system-ui, sans-serif; background:var(--bg); color:var(--text); margin:0; padding:20px; max-width: 600px; margin: auto; }
        .card { background:var(--card); padding:20px; border-radius:10px; border:1px solid #2a2a35; margin-bottom: 20px; }
        textarea { width: 100%; height: 200px; background: #0a0a0c; color: #8b8ba7; border: 1px solid #3d3d4e; border-radius: 6px; padding: 10px; font-family: monospace; font-size: 0.85rem; box-sizing: border-box; margin-bottom:15px;}
        .btn-group { display: flex; gap: 10px; margin-bottom: 20px; }
        button { flex: 1; border: none; padding: 12px 15px; border-radius: 6px; font-weight: bold; cursor: pointer; font-size: 0.95rem; display:flex; align-items:center; justify-content:center; gap:8px;}
        button:active { transform: scale(0.98); }
        .btn-export { background: #2979ff; color: #fff; }
        .btn-import { background: #3d3d4e; color: #fff; }
        .btn-apply { background: var(--accent); color: #121214; width: 100%; font-size: 1.1rem; padding:15px; }
        .btn-back { background: transparent; color: #8b8ba7; text-decoration: none; display: block; text-align: center; padding: 10px; border-radius: 6px; margin-top: 10px; font-weight: bold; border: 1px solid #3d3d4e;}
    </style>
</head>
<body>
    <div class="card">
        <h2>📦 Файли Налаштувань</h2>
        <div class="btn-group">
            <button class="btn-export" onclick="exportToFile()">💾 Зберегти на диск</button>
            <button class="btn-import" onclick="triggerFileInput()">📂 Завантажити з файлу</button>
            <input type="file" id="fileInput" accept=".json" style="display: none;" onchange="importFromFile(event)">
        </div>

        <h3 style="font-size: 0.9rem; color: #8b8ba7; margin-bottom: 5px;">Буфер JSON:</h3>
        <textarea id="jsonBox">%JSON_DATA%</textarea>
        
        <button class="btn-apply" onclick="applyConfig()">⚡ ЗАСТОСУВАТИ НА ESP32</button>
        <a href="/" class="btn-back">⬅ Повернутися до сканера</a>
    </div>

<script>
    function exportToFile() {
        try {
            let jsonText = document.getElementById('jsonBox').value;
            JSON.parse(jsonText); 
            let blob = new Blob([jsonText], { type: "application/json" });
            let url = URL.createObjectURL(blob);
            let a = document.createElement('a');
            a.href = url;
            a.download = "esp32_audio_config.json";
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(url);
        } catch(e) {
            alert('Помилка генерації файлу. Можливо пошкоджений формат JSON.');
        }
    }
    function triggerFileInput() { document.getElementById('fileInput').click(); }
    function importFromFile(event) {
        let file = event.target.files[0];
        if (!file) return;
        let reader = new FileReader();
        reader.onload = function(e) {
            try {
                let parsed = JSON.parse(e.target.result);
                document.getElementById('jsonBox').value = JSON.stringify(parsed, null, 2);
                alert('Файл прочитано! Натисніть "ЗАСТОСУВАТИ".');
            } catch (err) { alert('❌ Помилка читання: невірний JSON.'); }
            document.getElementById('fileInput').value = ""; 
        };
        reader.readAsText(file);
    }
    function applyConfig() {
        try {
            let rawJson = document.getElementById('jsonBox').value;
            let parsed = JSON.parse(rawJson); 
            fetch('/save_json_cfg', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(parsed)
            }).then(res => res.text()).then(result => {
                alert('✅ Оновлено!'); location.href = '/';
            }).catch(err => { alert('❌ Помилка мережі!'); });
        } catch(e) { alert('❌ Помилка формату JSON!'); }
    }
</script>
</body>
</html>
)rawliteral";

// ================= WEBSOCKET ОБРОБНИК =================
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED: {
            activeWebClients++;
            String out;
            getConfigJsonString(out);
            webSocket.sendTXT(num, out);
            break;
        }
        case WStype_DISCONNECTED:
            if (activeWebClients > 0) activeWebClients--;
            break;
        case WStype_TEXT: {
            JsonDocument doc;
            if (deserializeJson(doc, payload, length)) break;
            
            if (doc["type"] == "get_config") {
                String out;
                getConfigJsonString(out);
                webSocket.sendTXT(num, out);
            } else {
                processCommonCommand(doc);
            }
            break;
        }
        default: break;
    }
}

// ================= UART / RS485 ОБРОБНИК КОМАНД =================
void processUartCommand(String cmd) {
    if (cmd == "ACK") {
        lastRs232Ack = millis();
    } 
    else if (cmd.startsWith("{")) {
        // Якщо це JSON (ідентичний до WebSockets)
        JsonDocument doc;
        if (!deserializeJson(doc, cmd)) {
            if (doc["type"] == "get_config") {
                String out;
                getConfigJsonString(out);
                sendUart(out);
            } else {
                processCommonCommand(doc);
                needConfigBroadcast = true; // Оновлюємо WEB інтерфейс
            }
        }
    }
    else if (cmd.startsWith("ESC:")) {
        escTargetSpeed = cmd.substring(4).toInt();
    }
    else if (cmd.startsWith("CAM:")) {
        cameraPower = cmd.substring(4).toInt() > 0;
        ledcWrite(CAM_RELAY_PIN, cameraPower ? 410 : 205);
        saveGlobalCfgToNVS();
        needConfigBroadcast = true;
    }
    else if (cmd.startsWith("MUTE:")) {
        masterMute = cmd.substring(5).toInt() > 0;
        saveGlobalCfgToNVS();
        needConfigBroadcast = true;
    }
    else if (cmd == "STATUS") {
        char buf[128];
        sprintf(buf, "STATUS: ESC=%d CAM=%d MUTE=%d STATE=%d", escCurrentSpeed, cameraPower, masterMute, currentState);
        sendUart(String(buf));
    }
}

void handleUartCommands() {
    static String uartBuffer = "";
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {
            uartBuffer.trim();
            if (uartBuffer.length() > 0) {
                processUartCommand(uartBuffer);
            }
            uartBuffer = "";
        } else if (uartBuffer.length() < 1024) {
            uartBuffer += c;
        } else {
            uartBuffer = ""; // Захист від переповнення буфера
        }
    }
}

// ================= МЕРЕЖЕВІ МАРШРУТИ =================
void startSoftAP() {
    WiFi.mode(WIFI_AP);
    delay(100);
    WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    
    // Нова назва точки доступу
    if (WiFi.softAP("Wi-Fi Observer", "12345678", 1, 0, 4)) {
        dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));
        if (!serversStarted) {
            server.on("/", []() { server.send(200, "text/html", INDEX_HTML); });
            
            server.on("/config", []() {
                String jsonStr = "";
                getConfigJsonString(jsonStr);
                String page = FPSTR(CONFIG_HTML);
                page.replace("%JSON_DATA%", jsonStr);
                server.send(200, "text/html", page);
            });
            
            server.on("/save_json_cfg", HTTP_POST, []() {
                if (server.hasArg("plain")) {
                    JsonDocument doc;
                    if (!deserializeJson(doc, server.arg("plain"))) {
                        agcEnabled = doc["agc"] | agcEnabled;
                        manualGain = doc["gain"] | manualGain;
                        agcMinGain = doc["agc_min"] | agcMinGain;
                        agcMaxGain = doc["agc_max"] | agcMaxGain;
                        masterMute = doc["master_mute"] | masterMute;
                        escModuleEnabled = doc["esc_enabled"] | escModuleEnabled;
                        cameraPower = doc["cam_power"] | cameraPower;
                        
                        ledcWrite(CAM_RELAY_PIN, cameraPower ? 410 : 205);
                        saveGlobalCfgToNVS();
                        
                        JsonArray arr = doc["patterns"];
                        int idx = 0;
                        for (JsonObject p : arr) {
                            if (idx < NUM_PATTERNS) {
                                strlcpy(patterns[idx].name, p["name"] | patterns[idx].name, sizeof(patterns[idx].name));
                                patterns[idx].fMin = p["fMin"] | patterns[idx].fMin;
                                patterns[idx].fMax = p["fMax"] | patterns[idx].fMax;
                                patterns[idx].threshold = p["threshold"] | patterns[idx].threshold;
                                patterns[idx].minFrames = p["minFrames"] | patterns[idx].minFrames;
                                patterns[idx].isMuted = p["isMuted"] | patterns[idx].isMuted;
                                patterns[idx].baseLevel = p["baseLevel"] | patterns[idx].baseLevel;
                                
                                patterns[idx].triggerCount = 0;
                                patterns[idx].currentLevel = patterns[idx].baseLevel;
                                savePatternToNVS(idx);
                            }
                            idx++;
                        }
                        server.send(200, "text/plain", "OK");
                        return;
                    }
                }
                server.send(400, "text/plain", "ERROR");
            });

            server.onNotFound([]() {
                server.sendHeader("Location", "http://192.168.4.1/", true);
                server.send(302, "text/plain", "");
            });
            server.begin();
            webSocket.begin();
            webSocket.onEvent(onWebSocketEvent);
            serversStarted = true;
        }
    }
}

void stopSoftAP() {
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
}

// ================= I2S & DSP =================
void initI2S() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, NULL, &rx_handle);

    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
    slot_cfg.ws_width = 32; 
    slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, .bclk = I2S_BCLK_PIN, .ws = I2S_WS_PIN, 
            .dout = I2S_GPIO_UNUSED, .din = I2S_DIN_PIN,
            .invert_flags = {false, false, false}
        },
    };
    i2s_channel_init_std_mode(rx_handle, &std_cfg);
    i2s_channel_enable(rx_handle);
}

void initDSP() {
    dsps_fft2r_init_fc32(NULL, MAX_FFT_SIZE);
    dsps_wind_hann_f32(wind_func, FFT_SIZE);
}

// ================= ЛОГІКА АПАРАТНИХ ВУЗЛІВ =================
void handleHardware() {
    // Управління ESC колекторного мотора (50Hz, 12-bit = max 4095)
    // Нейтраль: 1500µs (307)
    if (escModuleEnabled) {
        if (escState == ESC_OFF) {
            // Вмикаємо оптопару та подаємо нейтраль 1500µs для ініціалізації
            digitalWrite(ESC_SW_PIN, HIGH);
            ledcWrite(ESC_PWM_PIN, 307); 
            escArmStartTime = millis();
            escState = ESC_ARMING;
            escTargetSpeed = 0;
            escCurrentSpeed = 0;
            escInPause = false;
        } else if (escState == ESC_ARMING) {
            if (millis() - escArmStartTime > 2000) {
                escState = ESC_READY; // Через 2 сек готовий до прийому швидкості
            }
        } else if (escState == ESC_READY) {
            
            // Якщо виконується пауза під час зміни напрямку
            if (escInPause) {
                ledcWrite(ESC_PWM_PIN, 307); // Примусова нейтраль
                if (millis() - escPauseTime > 800) { // Пауза 800 мс для надійного розблокування реверсу
                    escInPause = false;
                    escCurrentSpeed = escTargetSpeed; // Тільки тепер застосовуємо нову швидкість
                }
            } else {
                // Перевіряємо чи відбулась зміна знаку (з плюса на мінус або навпаки)
                if ((escCurrentSpeed > 0 && escTargetSpeed < 0) || 
                    (escCurrentSpeed < 0 && escTargetSpeed > 0)) {
                    
                    escInPause = true;
                    escPauseTime = millis();
                    escCurrentSpeed = 0;
                    ledcWrite(ESC_PWM_PIN, 307); // Переходимо в нейтраль
                    
                } else {
                    // Звичайна зміна швидкості без реверса
                    escCurrentSpeed = escTargetSpeed;
                    int duty = 307;
                    
                    if (escCurrentSpeed > 0) {
                        // Вперед: більш агресивна шпаруватість для впевненого старту
                        duty = map(escCurrentSpeed, 1, 100, 360, 410); 
                    } else if (escCurrentSpeed < 0) {
                        // Назад: більш агресивна шпаруватість для впевненого старту
                        duty = map(escCurrentSpeed, -1, -100, 254, 205); 
                    }
                    
                    ledcWrite(ESC_PWM_PIN, duty);
                }
            }
        }
    } else {
        if (escState != ESC_OFF) {
            ledcWrite(ESC_PWM_PIN, 307); // Скидаємо в нейтраль
            digitalWrite(ESC_SW_PIN, LOW); // Вимикаємо через оптопару
            escState = ESC_OFF;
            escTargetSpeed = 0;
            escCurrentSpeed = 0;
            escInPause = false;
        }
    }
}

// ================= ОБРОБКА АУДІО =================
void processAudio() {
    if (rx_handle == NULL) return;
    unsigned long now = millis();

    if (isMicSleeping) {
        if (now >= micSleepUntil) {
            isMicSleeping = false;
            i2s_channel_enable(rx_handle); 
            
            for(int p = 0; p < NUM_PATTERNS; p++) {
                patterns[p].triggerCount = 0;
                patterns[p].currentLevel = patterns[p].baseLevel;
            }
        } else {
            static unsigned long lastSleepSend = 0;
            if (now - lastSleepSend > 500 && serversStarted && activeWebClients > 0) {
                lastSleepSend = now;
                JsonDocument doc;
                doc["type"] = "sleep_status";
                doc["left"] = (micSleepUntil - now) / 1000;
                String out; serializeJson(doc, out);
                webSocket.broadcastTXT(out);
            }
            delay(10); 
            return;
        }
    }

    size_t bytes_read = 0;
    if (i2s_channel_read(rx_handle, raw_pcm, sizeof(raw_pcm), &bytes_read, portMAX_DELAY) != ESP_OK) return;

    float raw_sum_sq = 0.0f;
    static float dc_offset = 0.0f; 

    for (int i = 0; i < FFT_SIZE; i++) {
        float raw_sample = (float)(raw_pcm[i*2] >> 14) / 131072.0f;
        dc_offset = 0.995f * dc_offset + 0.005f * raw_sample;
        clean_samples[i] = raw_sample - dc_offset;
        raw_sum_sq += clean_samples[i] * clean_samples[i];
    }
    float raw_vu = sqrtf(raw_sum_sq / FFT_SIZE);

    if (agcEnabled) {
        if (raw_vu < 0.001f) {
            dynamicGain += (agcMinGain - dynamicGain) * 0.05f;
        } else {
            float current_vu = raw_vu * dynamicGain;
            float target_vu = 0.08f; 
            if (current_vu > target_vu * 1.3f) dynamicGain *= 0.92f; 
            else if (current_vu < target_vu * 0.7f) dynamicGain *= 1.02f;
        }
        dynamicGain = constrain(dynamicGain, agcMinGain, agcMaxGain); 
    } else {
        dynamicGain = manualGain;
    }

    float out_sum_sq = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++) {
        float sample = clean_samples[i] * dynamicGain;
        out_sum_sq += sample * sample;
        fft_input[i * 2]     = sample * wind_func[i];
        fft_input[i * 2 + 1] = 0.0f;
    }
    float final_vu = sqrtf(out_sum_sq / FFT_SIZE);

    dsps_fft2r_fc32(fft_input, FFT_SIZE);
    dsps_bit_rev_fc32(fft_input, FFT_SIZE);

    for (int i = 0; i < FFT_SIZE / 2; i++) {
        magnitude[i] = sqrtf(fft_input[i * 2] * fft_input[i * 2] + fft_input[i * 2 + 1] * fft_input[i * 2 + 1]) / FFT_SIZE;
    }

    if (recordingPatternId != -1) {
        recordFrameCount++;
        for (int i = 0; i < FFT_SIZE / 2; i++) accum_magnitude[i] += magnitude[i];

        if (now - recordStartTime > 2000) {
            int peak_bin = 4; 
            float peak_val = 0.0f;
            if (recordFrameCount > 0) {
                for (int i = 4; i < FFT_SIZE / 2; i++) {
                    accum_magnitude[i] /= recordFrameCount;
                    if (accum_magnitude[i] > peak_val) { peak_val = accum_magnitude[i]; peak_bin = i; }
                }
            }
            int lower_bin = peak_bin, upper_bin = peak_bin;
            while (lower_bin > 3 && accum_magnitude[lower_bin] > peak_val * 0.25f) lower_bin--;
            while (upper_bin < (FFT_SIZE / 2) - 2 && accum_magnitude[upper_bin] > peak_val * 0.25f) upper_bin++;
            if (upper_bin - lower_bin < 6) { lower_bin = max(3, peak_bin - 3); upper_bin = min(FFT_SIZE / 2 - 1, peak_bin + 4); }

            float band_energy = 0;
            for (int i = lower_bin; i <= upper_bin; i++) band_energy += accum_magnitude[i];

            patterns[recordingPatternId].fMin = (int)(lower_bin * 31.25f / 50.0f) * 50;
            patterns[recordingPatternId].fMax = (int)(upper_bin * 31.25f / 50.0f) * 50 + 50;
            patterns[recordingPatternId].threshold = max(0.015f, band_energy * 0.55f); 
            patterns[recordingPatternId].minFrames = 5; 
            
            savePatternToNVS(recordingPatternId);
            recordingPatternId = -1;
            needConfigBroadcast = true; 
        }
    }

    globalAlarm = false;

    for (int p = 0; p < NUM_PATTERNS; p++) {
        int bin_min = constrain(patterns[p].fMin / 31.25f, 0, (FFT_SIZE / 2) - 1);
        int bin_max = constrain(patterns[p].fMax / 31.25f, 0, (FFT_SIZE / 2) - 1);
        float band_energy = 0.0f;
        for (int i = bin_min; i <= bin_max; i++) band_energy += magnitude[i];
        
        patterns[p].currentEnergy = band_energy; 

        if (band_energy > patterns[p].threshold && !patterns[p].isMuted && !masterMute) {
            patterns[p].frameCounter++;
            
            if (patterns[p].frameCounter >= patterns[p].minFrames) {
                if (!patterns[p].isActive) {
                    patterns[p].isActive = true;
                    patterns[p].triggerCount++;
                    patterns[p].currentLevel = patterns[p].baseLevel + (patterns[p].triggerCount / 10);
                    
                    if (patterns[p].currentLevel >= 5) {
                        patterns[p].currentLevel = 5;
                        isMicSleeping = true;
                        micSleepUntil = now + 60000;
                        i2s_channel_disable(rx_handle);
                        
                        sendUart("SPAM DETECTED:" + String(patterns[p].name));
                        break; 
                    }

                    // Транслюємо подію в RS485
                    sendUart("ALARM:" + String(patterns[p].name) + " LVL:" + String(patterns[p].currentLevel));
                    
                    if (serversStarted && activeWebClients > 0) {
                        JsonDocument evt;
                        evt["type"] = "alarm_event";
                        evt["name"] = patterns[p].name;
                        evt["lvl"] = patterns[p].currentLevel;
                        String out; serializeJson(evt, out);
                        webSocket.broadcastTXT(out);
                    }
                }
                patterns[p].lastTriggerTime = now;
            }
        } else {
            patterns[p].frameCounter = 0;
            if (patterns[p].isActive && (now - patterns[p].lastTriggerTime > 1500)) {
                patterns[p].isActive = false; 
            }
        }
        if (patterns[p].isActive) globalAlarm = true;
    }

    if (globalAlarm) setRGB(255, 0, 0); 
    else if (currentState == STATE_FAILSAFE) setRGB(0, 0, 150); 
    else setRGB(0, constrain((uint8_t)(final_vu * 800.0f), 5, 180), 0); 

    static unsigned long lastWsSend = 0;
    if (serversStarted && activeWebClients > 0 && (now - lastWsSend > 66)) {
        lastWsSend = now;
        JsonDocument doc;
        doc["type"] = "fft";
        doc["vu"] = final_vu;
        doc["dyn_gain"] = dynamicGain; 
        doc["state"] = (currentState == STATE_NORMAL) ? "NORMAL" : ((currentState == STATE_FAILSAFE) ? "FAILSAFE" : "DEBUG");
        doc["rs232"] = (lastRs232Ack > 0 && (now - lastRs232Ack < RS232_TIMEOUT));
        
        doc["cam_pwr"] = cameraPower;
        doc["esc_state"] = (int)escState;
        
        doc["esc_target"] = escTargetSpeed;
        doc["esc_in_pause"] = escInPause;

        JsonArray actArr = doc["active_patterns"].to<JsonArray>();
        JsonArray enArr = doc["energies"].to<JsonArray>();
        JsonArray trigArr = doc["triggers"].to<JsonArray>();
        JsonArray lvlArr = doc["levels"].to<JsonArray>();
        
        for(int p=0; p<NUM_PATTERNS; p++) {
            actArr.add(patterns[p].isActive);
            enArr.add(patterns[p].currentEnergy);
            trigArr.add(patterns[p].triggerCount);
            lvlArr.add(patterns[p].currentLevel);
        }
        
        JsonArray data = doc["data"].to<JsonArray>();
        for (int i = 0; i < 64; i++) data.add(magnitude[i]);

        String jsonOut; serializeJson(doc, jsonOut);
        webSocket.broadcastTXT(jsonOut);
    }
}

// ================= СТАТУСНИЙ АВТОМАТ =================
void handleFailsafeLogic() {
    unsigned long now = millis();
    if (!firstCheckDone && now > 1000) firstCheckDone = true;

    bool rs232Alive = (lastRs232Ack > 0) && (now - lastRs232Ack < RS232_TIMEOUT);

    switch (currentState) {
        case STATE_NORMAL:
            if (!rs232Alive && firstCheckDone) { startSoftAP(); currentState = STATE_FAILSAFE; }
            else if (activeWebClients > 0) { currentState = STATE_DEBUG; }
            break;
        case STATE_FAILSAFE:
            if (rs232Alive && activeWebClients == 0) { stopSoftAP(); currentState = STATE_NORMAL; }
            break;
        case STATE_DEBUG:
            if (activeWebClients == 0) {
                if (rs232Alive) { stopSoftAP(); currentState = STATE_NORMAL; }
                else { currentState = STATE_FAILSAFE; }
            }
            break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, RS232_RX_PIN, RS232_TX_PIN);
    setRGB(50, 50, 0);
    
    // Ініціалізація апаратних пінів
    pinMode(ESC_SW_PIN, OUTPUT);
    digitalWrite(ESC_SW_PIN, LOW); // За замовчуванням оптопара вимкнена
    
    if (RS485_DE_PIN >= 0) {
        pinMode(RS485_DE_PIN, OUTPUT);
        digitalWrite(RS485_DE_PIN, LOW);
    }
    
    // LEDC таймери: ESC (50Hz) та Relay (50Hz) — стандарт для RC-сигналів
    ledcAttach(ESC_PWM_PIN, 50, 12);
    ledcAttach(CAM_RELAY_PIN, 50, 12);
    
    loadConfig();
    
    // Синхронізуємо реле камери після завантаження
    ledcWrite(CAM_RELAY_PIN, cameraPower ? 410 : 205);
    
    WiFi.mode(WIFI_OFF);
    initI2S();
    initDSP();
    setRGB(0, 0, 0);
}

void loop() {
    handleUartCommands();
    handleFailsafeLogic();
    handleHardware();
    
    if (serversStarted && WiFi.getMode() != WIFI_OFF) {
        dnsServer.processNextRequest(); 
        server.handleClient();
        webSocket.loop();
    }
    processAudio();
    
    if (needConfigBroadcast && serversStarted) {
        needConfigBroadcast = false;
        String out; getConfigJsonString(out);
        webSocket.broadcastTXT(out);
    }
}