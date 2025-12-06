// Привіт, я створюю свій проект, це под система, я її роблю для себе просто як свій проект.
// Я використовую ESP32C3, дисплей SSD1306 128x32, 5 сторонню кнопку для навігації(джойстик) барометр bmp180, датчик струму і напруги INA226, адресний світлодіод WS2812B.
// Так як це под система вейп девайс, я роблю основний екран з показниками: потужність, кількість затяжок, напруга акумулятора, відсоток заряду акумулятора, стан блютузу.
// Є ще 3 екрани меню: налаштування, статистика, інформація. Перемикання між екранами за допомогою кнопок вліво
// Трохи розповім про апаратну частину, для вейпа треба керувати великим струмом, тому я використовую MOSFET irlz44n, керую ним з піну мікроконтролера через резистор 220 Ом.
// Ще також на друкованій платі яку я створив в easyeda є дільник напруги на резисторах 43к і 75к для вимірювання напруги акумулятора через ADC мікроконтролера.
// Для вимірювання струму і напруги навантаження використовую INA226, він підключений по I2C шині разом з дисплеєм SSD1306 і барометром bmp180.
// цей текст я написав коментарем в код щоб ти завжди розумів про що йде мова тому що багато llm забувають контекст проекту коли бачать тільки код.
//ROLE : Ти — Expert Embedded Systems Engineer та UI / UX Designer, що спеціалізується на ESP32, графіці(LittlevGL / LVGL або Adafruit GFX) та системах реального часу(RTOS).PROJECT GOAL : Створити прошивку "PodikOS" для кастомного пристрою на базі ESP32 - C3 SuperMini.Система має бути візуально привабливою(стиль Material 3 / Expressive), з плавними анімаціями(60 FPS, Ease - Out), багатозадачною та енергоефективною.HARDWARE SPECIFICATION : MCU : ESP32 - C3 SuperMini.Display : OLED SSD1306(128x64 or 128x32) I2C.Sensors : INA226(I2C) — High - side моніторинг струму / напруги / потужності.BMP180(I2C) — Датчик тиску(використовується як датчик затяжки).MPU6050(I2C) — Гіроскоп / Акселерометр.Input : 5 - way Joystick(через PCF8574 I2C expander або прямі піни, якщо вистачить), Touch Pins(опціонально).Output : MOSFET(PWM Control) — нагрів спіралі.WS2812B(RGB LED) — статус та ефекти.IR LED + IR Receiver(на пінах RX / TX).Connectivity : BLE(для мобільного додатку), Wi - Fi(для OTA та WebSerial).SYSTEM ARCHITECTURE &FEATURES : 1. Power Control Engine("Vape Engine") : PID - Regulator : Використовувати зворотний зв'язок від INA226 для стабілізації потужності (Vari-Watt). Якщо напруга просідає, ШІМ (PWM) на MOSFET має автоматично збільшуватись. Pressure - Sensitive Mode : Режим, де потужність залежить від дельти тиску на BMP180(сильніша тяга = більше пару).Power Curves : Реалізувати систему кастомних графіків подачі потужності(Pre - heat, Curve).Інтерфейс редагування кривої : шкала часу(X)
//та відсотків потужності(Y).2. UI / UX &Graphics(Animation Engine) : Framework : Використовувати легку графічну бібліотеку, оптимізовану під monochrome OLED.Transitions : Всі переходи між меню та зміна значень мають використовувати функцію згладжування(Easing function : Ease - Out - Cubic або Ease - Out - Quad).Жодних різких змін кадрів.Fluid Simulation : Реалізувати 2D - симуляцію рідини(Metaballs або Particle System) на екрані зарядки.Рівень "води" відповідає % заряду.Гравітація рідини керується даними з MPU6050.Vape Animations : Під час затяжки(Fire)
//відтворювати анімації(з паралаксом від гіроскопа) : Vapor clouds(процедурний дим).Bubbles(бульбашки, що піднімаються).Popcorn(фанова анімація).3D GFX Demo : Рендеринг простих 3D - примітивів(wireframe куб, піраміда, торус), що обертаються відповідно до нахилу пристрою(MPU6050).3. RGB LED Effects(WS2812B) : Fade - in / Fade - out : Плавна зміна яскравості при будь - якій події.Light Painting Mode : При струшуванні пристрою(детекованому MPU6050) швидко змінювати колір LED, створюючи ефект малювання світлом у повітрі.Vape Modes : Rainbow, Random Color, Static Color, Breathing.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         5. Statistics &Info : Counters : Puff count,
   // Total vape time, Energy used(Joules &Watts last session), Cartridge lifespan tracker.Drop Counter : Використовувати акселерометр для детекції вільного падіння(Freefall detection) та удару.Інкрементувати лічильник "Drop Stats".Plug - in Counter : Лічильник підключень зарядного кабелю(детекція по різкому стрибку напруги на дільнику).Info Screen : OS Version, Device Owner, Serial Number, Uptime.Easter Egg : Міні - гра при багаторазовому натисканні на версію ПЗ.6. Settings Menu : Screen Brightness.Animation Speed.Sensor Raw Data(Gyro / Meteo view with smoothing option).LED Config.7. Data Management : Використовувати Preferences.h(NVS) для збереження всіх налаштувань, статистики та IR - кодів.Дані не повинні зникати при розряді АКБ.IMPLEMENTATION NOTES : Використовувати FreeRTOS для багатозадачності : Core 0(або окремий Task) : Рендеринг графіки та UI(низький пріоритет).Core 0(або окремий Task) : PID - регулятор потужності та зчитування сенсорів(високий пріоритет, критично для безпеки).Реалізувати Deep Sleep після таймауту неактивності.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "bitmaps.h"
#include "Jersey10_Regular20pt7b.h"
#include "Jersey10_Regular12pt7b.h"

// --- Налаштування пінів ---
#define PIN_LED 6 // Вбудований LED
#define PIN_BTN_M 7 // Center
#define PIN_BTN_F 4 // UP
#define PIN_BTN_B 3 // DOWN
#define PIN_BTN_R 2 // Right
#define PIN_BTN_L 1 // Left
#define PIN_MOSFET 20
#define PIN_SDA 8
#define PIN_SCL 9

// --- Налаштування PWM ---
#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// --- Налаштування 2D GFX ---
#define MAX_PARTICLES 30

bool isPuffing = false;
bool pwmActive = false;
bool isFireLocked = false; // Блокування кнопки Fire після тайм-ауту
unsigned long puffStartTime_ms = 0;
float puffPressureThreshold_Pa = 25.0f;

int fireUnlockPresses = 0;
bool showLockedMessage = false;
unsigned long lockedMessageEndTime = 0;



#define PIN_LED_WS2812B 21
#define LED_COUNT 1

// Нові налаштування для LED ефектів
enum LEDEffect {
    FX_RAINBOW,
    FX_RANDOM_PUFF,
    FX_STATIC_RED,
    FX_STATIC_GREEN,
    FX_STATIC_BLUE,
    FX_STATIC_PURPLE,
    FX_STATIC_YELLOW,
    FX_STATIC_WHITE,
    LED_EFFECT_COUNT
};
const char* ledEffectNames[] = {
    "Rainbow", "Random", "Red", "Green", "Blue", "Purple", "Yellow", "White"
};
LEDEffect currentLEDEffect = FX_RAINBOW;

uint32_t currentPuffColor = 0; // Колір для поточного затяжки (Random/Static)

// Налаштування для Light Painting
bool isShaking = false;
unsigned long lastShakeTime = 0;
#define SHAKE_THRESHOLD 30.0f // Поріг для детекції струсу (m/s^2), треба буде підібрати
#define SHAKE_TIMEOUT 250     // Тривалість ефекту після струсу (ms)


// --- Ініціалізація ---
Adafruit_SSD1306 display(128, 32, &Wire, -1);
Adafruit_MPU6050 mpu;
Preferences preferences;

static unsigned long lastDisplayMs = 0;
const uint8_t DISPLAY_FPS = 60; // поставити 30..60. 45 дає хороший компроміс
const unsigned long DISPLAY_INTERVAL_MS = 1000 / DISPLAY_FPS;
int puffTimeout_s = 10;


// --- Стани програми та екрани ---
enum ScreenState {
    STATE_MAIN_SCREEN,
    STATE_MENU_VIEW,
    STATE_SETTINGS_MENU,
    STATE_POWER_CURVE,
    STATE_GFX_PREVIEW,// Новий стан для превью
    STATE_3D_PREVIEW 
};

ScreenState currentState = STATE_MAIN_SCREEN;
const char* screenStateNames[] = {"MAIN", "MENU_VIEW", "SETTINGS", "POWER_CURVE", "GFX_PREVIEW", "3D_PREVIEW"}; // Для дебагу

// --- Меню ---
const int MAIN_MENU_COUNT = 4; // MAIN, SETTINGS, STATS, INFO

// --- Змінні ---
int wattage = 20;
int puffCount = 9999;
float voltage = 3.7;
int batteryPercent = 75;

// --- Структура для частинок ---
struct Particle {
    float x, y;
    float vx, vy;
    uint8_t layer; // 0=передній, 1=середній, 2=задній
    bool active;
};
Particle vaporParticles[MAX_PARTICLES];
float gyroParallaxOffset = 0.0f;

// --- Структура для метаболів (рідина) ---
struct Metaball {
    float x, y;
    float vx, vy;
    float radius;
    bool active;
};
#define MAX_METABALLS 8
Metaball metaballs[MAX_METABALLS];

// --- Структура для попкорну ---
struct Popcorn {
    float x, y;
    float vx, vy;
    float size;      // Поточний розмір
    float maxSize;   // Максимальний розмір перед вибухом
    float age;       // Вік частинки (0 до 1)
    bool active;
};
#define MAX_POPCORN 20
Popcorn popcornParticles[MAX_POPCORN];

// --- Структура для бульбашок ---
struct Bubble {
    float x, y;
    float vx, vy;
    float radius;
    bool active;
};
#define MAX_BUBBLES 25
Bubble bubbleParticles[MAX_BUBBLES];

// --- Логіка затяжки (Puff Logic) ---
Adafruit_BMP085 bmp;
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED_WS2812B, NEO_GRB + NEO_KHZ800);



enum PuffDetectorState { PUFF_IDLE, PUFF_ACTIVE };
PuffDetectorState puffDetectorState = PUFF_IDLE;

unsigned long lastPressureReadTime = 0;
const int PRESSURE_READING_COUNT = 10;
float pressureReadings[PRESSURE_READING_COUNT] = {0};
int pressureReadingIndex = 0;
float baselinePressure_Pa = 0;

// --- Анімації ---
float vaporAnimProgress = 0.0f;
unsigned long vaporAnimLastUpdate = 0;
float ledBrightness = 0.0f;

// --- Налаштування ---
enum GFXAnimation { GFX_VAPOR, GFX_BUBBLES, GFX_POPCORN, GFX_ANIM_COUNT };
const char* gfxAnimNames[] = {"Vapor", "Bubbles", "Popcorn"};
GFXAnimation currentGFXAnim = GFX_VAPOR;


// Новий enum для 3D фігур
enum ShapeType {
    SHAPE_CUBE,
    SHAPE_CONE,
    SHAPE_HEX_PRISM,
    SHAPE_CUBOID,
    SHAPE_OCTAHEDRON,
    SHAPE_SPHERE,
    SHAPE_COUNT
};
const char* shapeNames[] = {"Cube", "Cone", "Hex Prism", "Cuboid", "Octahedron", "Sphere"};
ShapeType current3DShape = SHAPE_CUBE;




const char *settingsItems[] = {"Brightness", "PID-Regulator", "Power Curve", "Sensitivity", "Puff Timeout", "2D GFX", "3D GFX", "LED Effect", "Exit"};
const int SETTINGS_ITEM_COUNT = 9;
int currentSettingsItem = 0;
int brightness = 100;
bool pidEnabled = true;

// --- Налаштування Power Curve ---
struct Point {
    float x; // 0.0 to 1.0 (час)
    float y; // 0.0 to 1.0 (потужність %)
};
Point powerCurvePoints[4];
int selectedCurvePoint = 0; // 0-3 для точок, 4 для налаштування часу
float curveTotalTime = 1.0f;

// --- Анімація ---
bool isAnimating = false;
unsigned long animStartTime;
const int ANIM_DURATION = 300; // ms

// Горизонтальна анімація меню
float animatedMenuPosition = 0.0f; // 0.0 = Main, 1.0 = Settings, etc.
float animStartMenuPos = 0.0f;
float animTargetMenuPos = 0.0f;

// Вертикальна анімація налаштувань
float animatedSettingsPosition = 0.0f;
float animStartSettingsPos = 0.0f;
float animTargetSettingsPos = 0.0f;
float animatedSettingsScroll = 0.0f; // Нова змінна для позиції скролу
float animStartSettingsScroll = 0.0f;
float animTargetSettingsScroll = 0.0f;


// --- Debounce кнопок ---
const uint8_t BTN_COUNT = 5;
const uint8_t btnPins[BTN_COUNT] = {PIN_BTN_M, PIN_BTN_F, PIN_BTN_B, PIN_BTN_R, PIN_BTN_L};
const char* btnNames[BTN_COUNT] = {"M", "F", "B", "R", "L"};
bool btnReading[BTN_COUNT];
bool btnStable[BTN_COUNT];
unsigned long btnLastChange[BTN_COUNT];
const unsigned long DEBOUNCE_MS = 40;

// --- Прототипи функцій ---
void handleButtons();
void updateAndDraw();
void clearAllParticles(); // Новий прототип
void drawScreenByIndex(int index, int xOffset);
void startMenuAnimation();
void startSettingsAnimation();
float easeOutQuint(float t);
void drawPowerCurveScreen();
void drawMainScreen(int xOffset);
void drawMenuItemScreen(const uint8_t *icon, const char *text, int xOffset);
void drawSettingsScreen(float selectorY, float yScrollOffset);
void readBMP180DataAndDetectPuff();
void    updatePWM();
   void updateVaporAnimation();
  void  updateLed();
  void drawVaporAnimation();
void draw3DPreview();


struct Point3D { float x, y, z; };
struct Edge { int p1, p2; };

// Змінні для обертання 3D фігур від гіроскопа
float objectPitch = 0;
float objectRoll = 0;

// Функції для 3D
Point3D rotateX(Point3D p, float angle);
Point3D rotateY(Point3D p, float angle);
Point3D rotateZ(Point3D p, float angle);
void projectAndDraw(const Point3D* vertices, int numVertices, const Edge* edges, int numEdges, float scale, float offsetX, float offsetY);





void endPuff() {
    if (!isPuffing) return; // Вже неактивна
    isPuffing = false;
    pwmActive = false;
    
    // Рахуємо затяжку, якщо вона була достатньо довгою
    unsigned long currentPuffDuration = millis() - puffStartTime_ms;
    if (currentPuffDuration >= 200) {
      puffCount++;
      // Тут можна додати збереження в Preferences
    }
    puffStartTime_ms = 0;
    Serial.println("Puff Ended");
}

void clearAllParticles() {
    for (int j = 0; j < MAX_PARTICLES; j++) vaporParticles[j].active = false;
    for (int j = 0; j < MAX_BUBBLES; j++) bubbleParticles[j].active = false;
    for (int j = 0; j < MAX_POPCORN; j++) popcornParticles[j].active = false;
    for (int j = 0; j < MAX_METABALLS; j++) metaballs[j].active = false;
}




Point3D rotateX(Point3D p, float angle) {
    float y = p.y * cos(angle) - p.z * sin(angle);
    float z = p.y * sin(angle) + p.z * cos(angle);
    return {p.x, y, z};
}

Point3D rotateY(Point3D p, float angle) {
    float x = p.x * cos(angle) + p.z * sin(angle);
    float z = -p.x * sin(angle) + p.z * cos(angle);
    return {x, p.y, z};
}

Point3D rotateZ(Point3D p, float angle) {
    float x = p.x * cos(angle) - p.y * sin(angle);
    float y = p.x * sin(angle) + p.y * cos(angle);
    return {x, y, p.z};
}

// Проекція 3D точки на 2D екран
void projectAndDraw(const Point3D* vertices, int numVertices, const Edge* edges, int numEdges, float scale, float offsetX, float offsetY) {
    Point3D transformedVertices[numVertices];
    for (int i = 0; i < numVertices; ++i) {
        transformedVertices[i] = vertices[i];
        
        // Застосовуємо обертання від гіроскопа
        transformedVertices[i] = rotateX(transformedVertices[i], objectPitch);
        transformedVertices[i] = rotateY(transformedVertices[i], objectRoll);
        
        // Проектуємо на 2D
        float perspective = 1 + transformedVertices[i].z / 100.0f; // Проста перспектива
        int projX = (int)(transformedVertices[i].x * scale * perspective + offsetX);
        int projY = (int)(transformedVertices[i].y * scale * perspective + offsetY);
        display.drawPixel(projX, projY, SSD1306_WHITE); // Малюємо вершини
    }

    for (int i = 0; i < numEdges; ++i) {
        int p1_idx = edges[i].p1;
        int p2_idx = edges[i].p2;

        Point3D p1 = transformedVertices[p1_idx];
        Point3D p2 = transformedVertices[p2_idx];

        // Проектуємо точки на 2D для малювання ліній
        float perspective1 = 1 + p1.z / 100.0f;
        float perspective2 = 1 + p2.z / 100.0f;

        int x1 = (int)(p1.x * scale * perspective1 + offsetX);
        int y1 = (int)(p1.y * scale * perspective1 + offsetY);
        int x2 = (int)(p2.x * scale * perspective2 + offsetX);
        int y2 = (int)(p2.y * scale * perspective2 + offsetY);

        display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
    }
}


// --- Дані 3D фігур ---

// Cube
const Point3D cubeVertices[] = {
    {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
    {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
};
const Edge cubeEdges[] = {
    {0,1},{1,2},{2,3},{3,0}, // Front face
    {4,5},{5,6},{6,7},{7,4}, // Back face
    {0,4},{1,5},{2,6},{3,7}  // Connecting edges
};
const int numCubeVertices = sizeof(cubeVertices) / sizeof(Point3D);
const int numCubeEdges = sizeof(cubeEdges) / sizeof(Edge);

// Cone (приблизне)
const Point3D coneVertices[] = {
    {0, 2, 0}, // Apex
    {-1.5, -1, 0}, {0, -1, 1.5}, {1.5, -1, 0}, {0, -1, -1.5} // Base points
};
const Edge coneEdges[] = {
    {0,1},{0,2},{0,3},{0,4}, // Apex to base
    {1,2},{2,3},{3,4},{4,1}  // Base edges
};
const int numConeVertices = sizeof(coneVertices) / sizeof(Point3D);
const int numConeEdges = sizeof(coneEdges) / sizeof(Edge);

// Hexagonal Prism
const Point3D hexPrismVertices[] = {
    {-1, -2, -0.5}, {1, -2, -0.5}, {2, -0, -0.5}, {1, 2, -0.5}, {-1, 2, -0.5}, {-2, 0, -0.5}, // Front face
    {-1, -2, 0.5}, {1, -2, 0.5}, {2, -0, 0.5}, {1, 2, 0.5}, {-1, 2, 0.5}, {-2, 0, 0.5}   // Back face
};
const Edge hexPrismEdges[] = {
    {0,1},{1,2},{2,3},{3,4},{4,5},{5,0}, // Front face
    {6,7},{7,8},{8,9},{9,10},{10,11},{11,6}, // Back face
    {0,6},{1,7},{2,8},{3,9},{4,10},{5,11}  // Connecting edges
};
const int numHexPrismVertices = sizeof(hexPrismVertices) / sizeof(Point3D);
const int numHexPrismEdges = sizeof(hexPrismEdges) / sizeof(Edge);


// Cuboid (прямокутник)
const Point3D cuboidVertices[] = {
    {-2, -1, -1}, {2, -1, -1}, {2, 1, -1}, {-2, 1, -1},
    {-2, -1, 1}, {2, -1, 1}, {2, 1, 1}, {-2, 1, 1}
};
const Edge cuboidEdges[] = {
    {0,1},{1,2},{2,3},{3,0}, // Front face
    {4,5},{5,6},{6,7},{7,4}, // Back face
    {0,4},{1,5},{2,6},{3,7}  // Connecting edges
};
const int numCuboidVertices = sizeof(cuboidVertices) / sizeof(Point3D);
const int numCuboidEdges = sizeof(cuboidEdges) / sizeof(Edge);

// Octahedron
const Point3D octahedronVertices[] = {
    {0,0,2}, // Top apex
    {2,0,0}, {0,2,0}, {-2,0,0}, {0,-2,0}, // Middle square
    {0,0,-2} // Bottom apex
};
const Edge octahedronEdges[] = {
    {0,1},{0,2},{0,3},{0,4}, // Top pyramid
    {5,1},{5,2},{5,3},{5,4}, // Bottom pyramid
    {1,2},{2,3},{3,4},{4,1}  // Middle square
};
const int numOctahedronVertices = sizeof(octahedronVertices) / sizeof(Point3D);
const int numOctahedronEdges = sizeof(octahedronEdges) / sizeof(Edge);

// Sphere (апроксимація, icosahedron)
const float phi = (1.0f + sqrt(5.0f)) / 2.0f;
const Point3D sphereVertices[] = { // Icosahedron vertices
    {-1,  phi,  0}, { 1,  phi,  0}, {-1, -phi,  0}, { 1, -phi,  0},
    { 0, -1,  phi}, { 0,  1,  phi}, { 0, -1, -phi}, { 0,  1, -phi},
    { phi,  0, -1}, { phi,  0,  1}, {-phi,  0, -1}, {-phi,  0,  1}
};
const Edge sphereEdges[] = { // Icosahedron edges
    {0,1},{0,5},{0,7},{0,10},{0,11},
    {1,5},{1,7},{1,8},{1,9},
    {2,3},{2,4},{2,6},{2,10},{2,11},
    {3,4},{3,6},{3,8},{3,9},
    {4,5},{4,9},{4,11},
    {5,7},{5,11},
    {6,7},{6,8},{6,10},
    {8,9},{8,10},
    {9,11},
    {10,11}
};
const int numSphereVertices = sizeof(sphereVertices) / sizeof(Point3D);
const int numSphereEdges = sizeof(sphereEdges) / sizeof(Edge);

void updatePWM() {
    if (!isPuffing || !pwmActive) {
        ledcWrite(PWM_CHANNEL, 0);
        return;
    }

    unsigned long elapsed_ms = millis() - puffStartTime_ms;

    // 1. Перевірка на тайм-аут затяжки
    if (elapsed_ms >= (puffTimeout_s * 1000)) {
        Serial.println("Puff Timeout Reached! Locking fire button.");
        endPuff();
        isFireLocked = true; // Блокуємо до відпускання кнопки
        // Можна додати візуальний фідбек, наприклад, блимання екраном або вібрацію
        return;
    }

    // 2. Розрахунок прогресу по кривій
    float timeProgress = (float)elapsed_ms / (curveTotalTime * 1000.0f);
    if (timeProgress > 1.0f) {
        timeProgress = 1.0f; // Обмежуємо прогрес, щоб не вийти за межі кривої
    }

    float powerCurveMultiplier = 0.0f;

    // 3. Інтерполяція потужності по сегментах кривої
    if (timeProgress <= powerCurvePoints[0].x) {
        powerCurveMultiplier = powerCurvePoints[0].y;
    } else {
        for (int i = 0; i < 3; ++i) {
            if (timeProgress >= powerCurvePoints[i].x && timeProgress <= powerCurvePoints[i+1].x) {
                float segmentDuration = powerCurvePoints[i+1].x - powerCurvePoints[i].x;
                float segmentProgress = (segmentDuration == 0) ? 1.0f : (timeProgress - powerCurvePoints[i].x) / segmentDuration;
                powerCurveMultiplier = powerCurvePoints[i].y + (powerCurvePoints[i+1].y - powerCurvePoints[i].y) * segmentProgress;
                break;
            }
        }
    }
    
    // 4. Розрахунок кінцевої потужності (поки що без INA226, просто % від максимуму)
    // Максимальна потужність 200W (як вказано в UI)
    float wattageMultiplier = (float)wattage / 200.0f;
    
    float finalPowerMultiplier = powerCurveMultiplier * wattageMultiplier;
    finalPowerMultiplier = constrain(finalPowerMultiplier, 0.0f, 1.0f); // Захист від перевищення

    // 5. Перетворення потужності в значення ШІМ і відправка на MOSFET
    int finalPwmValue = (int)(finalPowerMultiplier * 255);
    ledcWrite(PWM_CHANNEL, finalPwmValue);
}

// --- Загальна функція для малювання поточної 3D фігури ---
void draw3DPreview() {
    display.clearDisplay(); // Очищуємо екран для 3D
    float scale = 6.0f; // Масштаб фігур
    float offsetX = display.width() / 2;
    float offsetY = display.height() / 2;

    switch (current3DShape) {
        case SHAPE_CUBE:
            projectAndDraw(cubeVertices, numCubeVertices, cubeEdges, numCubeEdges, scale, offsetX, offsetY);
            break;
        case SHAPE_CONE:
            projectAndDraw(coneVertices, numConeVertices, coneEdges, numConeEdges, scale, offsetX, offsetY);
            break;
        case SHAPE_HEX_PRISM:
            projectAndDraw(hexPrismVertices, numHexPrismVertices, hexPrismEdges, numHexPrismEdges, scale, offsetX, offsetY);
            break;
        case SHAPE_CUBOID:
            projectAndDraw(cuboidVertices, numCuboidVertices, cuboidEdges, numCuboidEdges, scale, offsetX, offsetY);
            break;
        case SHAPE_OCTAHEDRON:
            projectAndDraw(octahedronVertices, numOctahedronVertices, octahedronEdges, numOctahedronEdges, scale, offsetX, offsetY);
            break;
        case SHAPE_SPHERE:
            projectAndDraw(sphereVertices, numSphereVertices, sphereEdges, numSphereEdges, scale/phi, offsetX, offsetY); // Сфера трохи іншого масштабу
            break;
        default:
            break;
    }
    // Можна додати текст з назвою фігури
    display.setFont(NULL);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(shapeNames[current3DShape], 0, 0, &x1, &y1, &w, &h);
    display.setCursor((display.width() - w) / 2, display.height() - h - 1);
    display.print(shapeNames[current3DShape]);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n--- PodikOS Booting ---");
    // ...existing code...
    // Ініціалізуємо всі частинки як неактивні
    for (int i = 0; i < MAX_PARTICLES; i++) {
        vaporParticles[i].active = false;
    }
    for (int i = 0; i < MAX_METABALLS; i++) {
        metaballs[i].active = false;
    }
    for (int i = 0; i < MAX_POPCORN; i++) {
        popcornParticles[i].active = false;
    }
    for (int i = 0; i < MAX_BUBBLES; i++) {
        bubbleParticles[i].active = false;
    }

    // Ініціалізація точок кривої потужності (x, y)
// ...existing code...
    pinMode(PIN_BTN_M, INPUT_PULLUP);
    pinMode(PIN_BTN_F, INPUT_PULLUP);
    pinMode(PIN_BTN_B, INPUT_PULLUP);
    pinMode(PIN_BTN_R, INPUT_PULLUP);
    pinMode(PIN_BTN_L, INPUT_PULLUP);

    for (uint8_t i = 0; i < BTN_COUNT; ++i) {
        btnReading[i] = digitalRead(btnPins[i]);
        btnStable[i] = btnReading[i];
        btnLastChange[i] = millis();
    }

    Wire.begin(PIN_SDA, PIN_SCL);
    

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.clearDisplay();
    display.display();
    // Використовуємо глобальну змінну brightness для встановлення початкової яскравості
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(map(brightness, 0, 100, 0, 255));

    // Ініціалізація гіроскопа MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
    } else {
        Serial.println("MPU6050 Found!");
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // Ініціалізація ШІМ
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOSFET, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);

    // Ініціалізація датчика тиску
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    }

    // Ініціалізація світлодіода
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'

    // Ініціалізуємо всі частинки як неактивні
    for (int i = 0; i < MAX_PARTICLES; i++) {
        vaporParticles[i].active = false;
    }

    // Ініціалізація точок кривої потужності (x, y)
    powerCurvePoints[0] = {0.0f, 0.2f};
    powerCurvePoints[1] = {0.25f, 0.4f};
    powerCurvePoints[2] = {0.6f, 0.8f};
    powerCurvePoints[3] = {1.0f, 1.0f}; // Остання точка зафіксована по Y

    Serial.printf("Initial state: %s\n", screenStateNames[currentState]);
}

// --- Нові функції для 2D GFX ---
void spawnParticle(Particle &p) {
    p.x = random(0, display.width()); // З'являються по всій ширині
    p.y = display.height() + 5; // Знизу за екраном
    p.vx = random(-20, 20) / 100.0f; // Легкий рух в сторони
    p.vy = random(-80, -40) / 100.0f; // Повільний рух вгору
    p.layer = random(0, 3); // 0, 1 або 2
    p.active = true;
}

void updateParticles() {
    // Оновлюємо позицію кожної активної частинки
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (vaporParticles[i].active) {
            vaporParticles[i].x += vaporParticles[i].vx;
            vaporParticles[i].y += vaporParticles[i].vy;

            // Якщо затяжка не активна, частинки повільно розсіюються
            if (!isPuffing) {
                vaporParticles[i].vy *= 0.98f; // Сповільнюємо рух вгору
            }

            // Якщо частинка вилетіла за екран або майже зупинилась, деактивуємо її
            if (vaporParticles[i].y < -5 || abs(vaporParticles[i].vy) < 0.01f) {
                vaporParticles[i].active = false;
            }
        }
    }

    // Якщо затяжка активна, створюємо нові частинки
    if (isPuffing && vaporAnimProgress > 0.1f) {
        // Чим сильніша "затяжка" (прогрес анімації), тим більше шансів створити частинку
        if (random(0, 100) < vaporAnimProgress * 50) { 
            for (int i = 0; i < MAX_PARTICLES; i++) {
                if (!vaporParticles[i].active) {
                    spawnParticle(vaporParticles[i]);
                    break; // Створюємо одну частинку за раз
                }
            }
        }
    }
}
// ...existing code...
void updateGyro() {
    static unsigned long lastGyroRead = 0;
    if (millis() - lastGyroRead < 20) { // Читаємо не частіше, ніж раз на 20 мс (50 Гц)
        return;
    }
    lastGyroRead = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float accelerationMagnitude = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
    if (accelerationMagnitude > SHAKE_THRESHOLD) {
        isShaking = true;
        lastShakeTime = millis();
    }

    // Оновлюємо offset для 2D паралаксу
    float targetParallaxOffset = constrain(a.acceleration.x, -5.0, 5.0) * -1.0f;
    gyroParallaxOffset = gyroParallaxOffset * 0.9f + targetParallaxOffset * 0.1f;

    // Оновлюємо Pitch та Roll для 3D об'єктів
    // Перетворення акселерометра в кути (просте, без фільтрації Калмана)
    float accPitch = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z));
    float accRoll = atan2(-a.acceleration.x, a.acceleration.z);

    // Плавне оновлення кутів (комплементарний фільтр або просто згладжування)
    // Тут простий згладжувач
    objectPitch = objectPitch * 0.9f + accPitch * 0.1f;
    objectRoll = objectRoll * 0.9f + accRoll * 0.1f;
}
// ...existing code...

void drawParticles() {
    

    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (vaporParticles[i].active) {
            Particle &p = vaporParticles[i];
            float parallaxMultiplier = 0.0f;
            
            // Різні шари рухаються з різною швидкістю від нахилу
            if (p.layer == 0) parallaxMultiplier = 1.0f; // Передній план
            if (p.layer == 1) parallaxMultiplier = 0.6f; // Середній план
            if (p.layer == 2) parallaxMultiplier = 0.3f; // Задній план

            int screenX = p.x + (gyroParallaxOffset * parallaxMultiplier);
            int screenY = p.y;
            
            // Різні шари мають різний розмір
            if (p.layer == 0) display.fillCircle(screenX, screenY, 2, SSD1306_WHITE);
            else if (p.layer == 1) display.fillCircle(screenX, screenY, 1, SSD1306_WHITE);
            else display.drawPixel(screenX, screenY, SSD1306_WHITE);
        }
    }
}

void spawnMetaball(Metaball &m) {
    m.x = random(10, display.width() - 10);
    m.y = display.height() + 5;
    m.vx = random(-15, 15) / 100.0f;
    m.vy = random(-60, -30) / 100.0f;
    m.radius = random(2, 4);
    m.active = true;
}

void updateMetaballs() {
    for (int i = 0; i < MAX_METABALLS; i++) {
        if (metaballs[i].active) {
            metaballs[i].x += metaballs[i].vx;
            metaballs[i].y += metaballs[i].vy;
            metaballs[i].vy += 0.08f; // Гравітація

            if (!isPuffing) {
                metaballs[i].vy *= 0.97f;
            }

            if (metaballs[i].y > display.height() + 5) {
                metaballs[i].active = false;
            }
        }
    }

    if (isPuffing && vaporAnimProgress > 0.1f) {
        if (random(0, 100) < vaporAnimProgress * 40) {
            for (int i = 0; i < MAX_METABALLS; i++) {
                if (!metaballs[i].active) {
                    spawnMetaball(metaballs[i]);
                    break;
                }
            }
        }
    }
}

void drawMetaballs() {
   

    for (int i = 0; i < MAX_METABALLS; i++) {
        if (metaballs[i].active) {
            Metaball &m = metaballs[i];
            float parallaxMult = 0.5f; // Метаболи рухаються з середньою швидкістю
            int screenX = m.x + (gyroParallaxOffset * parallaxMult);
            int screenY = m.y;
            
            display.fillCircle(screenX, screenY, m.radius, SSD1306_WHITE);
        }
    }
}

// --- Функції для Попкорну ---
void spawnPopcorn(Popcorn &p) {
    p.x = random(5, display.width() - 5); // З'являється будь-де на екрані по X
    p.y = random(5, display.height() - 5); // і по Y
    p.vx = 0; // Не рухається
    p.vy = 0; // Не рухається
    p.size = 0.5f;
    p.maxSize = random(3, 6);
    p.age = 0.0f;
    p.active = true;
}

void updatePopcorn() {
    for (int i = 0; i < MAX_POPCORN; i++) {
        if (popcornParticles[i].active) {
            Popcorn &p = popcornParticles[i];
            // Прибираємо фізику: p.x += p.vx; p.y += p.vy; p.vy += 0.1f;

            p.age += 0.015f;

            // Фаза 1: Зростання (0 - 0.6)
            if (p.age < 0.6f) {
                p.size = p.maxSize * easeOutQuint(p.age / 0.6f);
            } 
            // Фаза 2: Вибух та розсіювання (0.6 - 1.0)
            else {
                p.size = p.maxSize * (2.0f - (p.age - 0.6f) / 0.4f * 2.0f);
                if (p.size < 0.2f) p.size = 0.2f;
            }

            if (p.age >= 1.0f) {
                p.active = false;
            }
        }
    }

    if (isPuffing && vaporAnimProgress > 0.15f) {
        if (random(0, 100) < vaporAnimProgress * 60) {
            for (int i = 0; i < MAX_POPCORN; i++) {
                if (!popcornParticles[i].active) {
                    spawnPopcorn(popcornParticles[i]);
                    break;
                }
            }
        }
    }
}

void drawPopcorn() {
    

    for (int i = 0; i < MAX_POPCORN; i++) {
        if (popcornParticles[i].active) {
            Popcorn &p = popcornParticles[i];
            float parallaxMult = 0.4f;
            int screenX = p.x + (gyroParallaxOffset * parallaxMult);
            int screenY = p.y;
            
            display.fillCircle(screenX, screenY, (int)p.size, SSD1306_WHITE);
        }
    }
}

// --- Функції для Бульбашок (контур) ---
void spawnBubble(Bubble &b) {
    b.x = random(10, display.width() - 10);
    b.y = display.height() + 5;
    b.vx = random(-25, 25) / 100.0f;
    b.vy = random(-90, -50) / 100.0f;
    b.radius = random(2, 4);
    b.active = true;
}

void updateBubbles() {
    for (int i = 0; i < MAX_BUBBLES; i++) {
        if (bubbleParticles[i].active) {
            Bubble &b = bubbleParticles[i];
            b.x += b.vx;
            b.y += b.vy;

            if (!isPuffing) {
                b.vy *= 0.98f;
            }

            if (b.y < -5) {
                b.active = false;
            }
        }
    }

    if (isPuffing && vaporAnimProgress > 0.1f) {
        if (random(0, 100) < vaporAnimProgress * 45) {
            for (int i = 0; i < MAX_BUBBLES; i++) {
                if (!bubbleParticles[i].active) {
                    spawnBubble(bubbleParticles[i]);
                    break;
                }
            }
        }
    }
}

void drawBubbles() {
    

    for (int i = 0; i < MAX_BUBBLES; i++) {
        if (bubbleParticles[i].active) {
            Bubble &b = bubbleParticles[i];
            float parallaxMult = 0.7f;
            int screenX = b.x + (gyroParallaxOffset * parallaxMult);
            int screenY = b.y;
            
            // Малюємо тільки контур (окружність)
            display.drawCircle(screenX, screenY, b.radius, SSD1306_WHITE);
        }
    }
}

// --- Комбінована функція для превью ---
void updateAnimationPreview() {
    if (currentGFXAnim == GFX_VAPOR) {
        updateParticles();
    } else if (currentGFXAnim == GFX_BUBBLES) {
        updateBubbles();
    } else if (currentGFXAnim == GFX_POPCORN) {
        updatePopcorn();
    }
}

void drawAnimationPreview() {
    if (currentGFXAnim == GFX_VAPOR) {
        drawParticles();
    } else if (currentGFXAnim == GFX_BUBBLES) {
        drawBubbles();
    } else if (currentGFXAnim == GFX_POPCORN) {
        drawPopcorn();
    }
}
// ...existing code...
void loop() {
    handleButtons();
    readBMP180DataAndDetectPuff();
    updateGyro(); // Зчитуємо гіроскоп в кожному циклі
    
    // Оновлюємо 2D анімації в залежності від поточного вибору
    // (Ця функція внутрішньо викликає updateParticles, updateBubbles, updatePopcorn)
    updateAnimationPreview(); 
    
    updatePWM();
    updateVaporAnimation(); // Це для загального прогресу анімації, що керує появою частинок
    updateLed();
    updateAndDraw();
    
}

// --- Анімація та рендеринг ---


float easeOutQuint(float t) {
    return 1 - pow(1 - t, 5);
}

void startMenuAnimation() {
    isAnimating = true;
    animStartTime = millis();
    animStartMenuPos = animatedMenuPosition; // Починаємо з поточної візуальної позиції
}

void startSettingsAnimation() {
    isAnimating = true;
    animStartTime = millis();
    animStartSettingsPos = animatedSettingsPosition;
    animStartSettingsScroll = animatedSettingsScroll; // Запам'ятовуємо початковий скрол
}






// --- Централізовані функції керування затяжкою ---
void startPuff() {
    if (isFireLocked) {
        fireUnlockPresses++;
        Serial.printf("Fire locked. Press count: %d\n", fireUnlockPresses);

        // Показуємо повідомлення на екрані на 2 секунди
        showLockedMessage = true;
        lockedMessageEndTime = millis() + 2000;

        if (fireUnlockPresses >= 5) {
            isFireLocked = false;
            fireUnlockPresses = 0;
            showLockedMessage = false; // Одразу ховаємо повідомлення
            Serial.println("Fire unlocked!");
            // Тут можна буде додати коротке повідомлення "Unlocked"
        }
        return; // Не починаємо затяжку, бо ми в режимі розблокування
    }

    if (isPuffing) return; // Стара логіка: не починати, якщо вже активна

    switch(currentLEDEffect) {
        case FX_RANDOM_PUFF:
            currentPuffColor = strip.ColorHSV(random(0, 65535));
            break;
        case FX_STATIC_RED:    currentPuffColor = strip.Color(255, 0, 0); break;
        case FX_STATIC_GREEN:  currentPuffColor = strip.Color(0, 255, 0); break;
        case FX_STATIC_BLUE:   currentPuffColor = strip.Color(0, 0, 255); break;
        case FX_STATIC_PURPLE: currentPuffColor = strip.Color(128, 0, 255); break;
        case FX_STATIC_YELLOW: currentPuffColor = strip.Color(255, 255, 0); break;
        case FX_STATIC_WHITE:  currentPuffColor = strip.Color(255, 255, 255); break;
        default: break; // Для Rainbow колір генерується динамічно
    }

    isPuffing = true;
    pwmActive = true;
    puffStartTime_ms = millis();
    Serial.println("Puff Started");
}





void updateAndDraw() {
    if (isAnimating) {
        unsigned long elapsedTime = millis() - animStartTime;
        if (elapsedTime >= ANIM_DURATION) {
            isAnimating = false;
            // Прив'язуємо до кінцевої позиції
            animatedMenuPosition = animTargetMenuPos;
            animatedSettingsPosition = animTargetSettingsPos;
            animatedSettingsScroll = animTargetSettingsScroll;
        } else {
            float progress = (float)elapsedTime / ANIM_DURATION;
            float easedProgress = easeOutQuint(progress);
            if (currentState == STATE_MENU_VIEW || currentState == STATE_MAIN_SCREEN) {
                 animatedMenuPosition = animStartMenuPos + (animTargetMenuPos - animStartMenuPos) * easedProgress;
            } else if (currentState == STATE_SETTINGS_MENU) {
                animatedSettingsPosition = animStartSettingsPos + (animTargetSettingsPos - animStartSettingsPos) * easedProgress;
                animatedSettingsScroll = animStartSettingsScroll + (animTargetSettingsScroll - animStartSettingsScroll) * easedProgress;
            }
        }
    }

    display.clearDisplay();

    if (currentState == STATE_SETTINGS_MENU) {
        float yScrollOffset = -round(animatedSettingsScroll * 11);
        float selectorY = (animatedSettingsPosition - animatedSettingsScroll) * 11;
        drawSettingsScreen(selectorY, yScrollOffset);
    } else if (currentState == STATE_POWER_CURVE) {
        drawPowerCurveScreen();
    } else if (currentState == STATE_GFX_PREVIEW) {
        // показ прев'ю на весь екран
        drawAnimationPreview();
    } else if (currentState == STATE_3D_PREVIEW) { // НОВИЙ СТАН
        draw3DPreview();
    }
    else {
        int prevScreenIndex = floor(animatedMenuPosition);
        int nextScreenIndex = ceil(animatedMenuPosition);
        float fraction = animatedMenuPosition - prevScreenIndex;
        if (fraction == 0.0f) {
            drawScreenByIndex(prevScreenIndex, 0);
        } else {
            int xOffset = -fraction * 128;
            drawScreenByIndex(prevScreenIndex, xOffset);
            drawScreenByIndex(nextScreenIndex, xOffset + 128);
        }
    }
    

    unsigned long now = millis();
    if (now - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
        display.display();
        lastDisplayMs = now;
    }
}

void drawScreenByIndex(int index, int xOffset) {
    // Обробка зациклювання індексів
    index = (index % MAIN_MENU_COUNT + MAIN_MENU_COUNT) % MAIN_MENU_COUNT;

    switch (index) {
        case 0: // MENU_MAIN
            drawMainScreen(xOffset);
            break;
        case 1: // MENU_SETTINGS
            drawMenuItemScreen(settings_icon, "Settings", xOffset);
            break;
        case 2: // MENU_STATS
            drawMenuItemScreen(statistics_icon, "Statistics", xOffset);
            break;
        case 3: // MENU_INFO
            drawMenuItemScreen(information_icon, "Information", xOffset);
            break;
    }
}

void drawMainScreen(int xOffset) {
    display.setTextColor(SSD1306_WHITE);
    int16_t x1, y1;
    uint16_t w, h;
    const int16_t top_margin = 2;

    display.setFont(&Jersey10_Regular12pt7b);
    char puffStr[12];
    sprintf(puffStr, "%d", puffCount);
    display.getTextBounds(puffStr, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(xOffset + 2, 8);
    display.print(puffStr);

    char voltStr[8];
    dtostrf(voltage, 3, 1, voltStr);
    strcat(voltStr, "V");
    display.getTextBounds(voltStr, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(xOffset + display.width() - w - 20, 8);
    display.print(voltStr);

    const uint8_t *batteryIcon;
    if (batteryPercent > 85) batteryIcon = battery_100;
    else if (batteryPercent > 60) batteryIcon = battery_75;

    else if (batteryPercent > 5) batteryIcon = battery_15;
    else batteryIcon = battery_0;
    display.drawBitmap(xOffset + display.width() - 18, 0, batteryIcon, 18, 9, SSD1306_WHITE);

    display.setFont(&Jersey10_Regular20pt7b);
    char wattStr[8];
    sprintf(wattStr, "%dW", wattage);
    display.getTextBounds(wattStr, 0, 0, &x1, &y1, &w, &h);
    int16_t x_center = (display.width() - w) / 2;
    int16_t area_top = top_margin + 8;
    int16_t area_height = display.height() - area_top - 2;
    int16_t y_top_for_text = area_top + (area_height - h) / 2;
    display.setCursor(xOffset + x_center, y_top_for_text - y1);
    display.print(wattStr);
    
    // Замінюємо стару анімацію на нову систему частинок
   // if (currentGFXAnim == GFX_VAPOR) {
    //    drawParticles();
   // }
    drawAnimationPreview();

    if (showLockedMessage && millis() < lockedMessageEndTime) {
        display.fillRoundRect(10, 2, 108, 28, 4, SSD1306_BLACK); // Чорний фон
        display.drawRoundRect(10, 2, 108, 28, 4, SSD1306_WHITE); // Біла рамка
        
        
        display.setFont(NULL);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        
        display.setCursor(28, 6);
        display.print("Fire Locked!");
        
        char countStr[16];
        sprintf(countStr, "Press %d more", 5 - fireUnlockPresses);
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(countStr, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((display.width() - w) / 2, 18);
        display.print(countStr);

    } else if (showLockedMessage && millis() >= lockedMessageEndTime) {
        showLockedMessage = false; // Повідомлення зникло по таймеру
        fireUnlockPresses = 0; // Скидаємо лічильник, щоб не було плутанини
    }
}

void drawMenuItemScreen(const uint8_t *icon, const char *text, int xOffset) {
    display.setTextColor(SSD1306_WHITE);
    if (icon) {
        display.drawBitmap(xOffset + (display.width() - 24) / 2, 2, icon, 16, 16, SSD1306_WHITE);
    }
    display.setFont(&Jersey10_Regular12pt7b);
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(xOffset + (display.width() - w) / 2, 2 + 16 + 4 - y1);
    display.print(text);
}

void drawPowerCurveScreen() {
    display.setTextColor(SSD1306_WHITE);
    display.setFont(NULL);
    display.setTextSize(1);

    // --- Область графіка ---
    const int graphX = 8, graphY = 2, graphW = 100, graphH = 26;

    // --- Малюємо осі ---
    display.drawLine(graphX, graphY, graphX, graphY + graphH, SSD1306_WHITE); // Вісь Y
    display.drawLine(graphX, graphY + graphH, graphX + graphW - 25, graphY + graphH, SSD1306_WHITE); // Вісь X (коротша)

    // --- Малюємо мітку часу ---
    char timeStr[8];
    // Форматуємо рядок, щоб показувати ".1" для чисел < 1.0 і "1.5" для > 1.0
    dtostrf(curveTotalTime, (curveTotalTime < 1.0f && curveTotalTime > 0.0f) ? 1 : 2, 1, timeStr);
    strcat(timeStr, "S");
    
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);
    
    int timeTextX = graphX + graphW - 20;
    int timeTextY = graphY + graphH - 7;
    
    if (selectedCurvePoint == 4) { // Якщо вибрано редагування часу
        display.fillRoundRect(timeTextX - 2, timeTextY - 1, w + 4, h + 2, 2, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
    } else {
        display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(timeTextX, timeTextY);
    display.print(timeStr);

    // --- Малюємо криву та точки (код без змін) ---
    for (int i = 0; i < 4; i++) {
        int px = graphX + powerCurvePoints[i].x * graphW;
        int py = (graphY + graphH) - powerCurvePoints[i].y * graphH;
        if (i > 0) {
            int prev_px = graphX + powerCurvePoints[i - 1].x * graphW;
            int prev_py = (graphY + graphH) - powerCurvePoints[i - 1].y * graphH;
            display.drawLine(prev_px, prev_py, px, py, SSD1306_WHITE);
        }
    }
    for (int i = 0; i < 4; i++) {
        int px = graphX + powerCurvePoints[i].x * graphW;
        int py = (graphY + graphH) - powerCurvePoints[i].y * graphH;
        if (i == selectedCurvePoint) {
            display.fillCircle(px, py, 3, SSD1306_WHITE);
        } else {
            display.fillCircle(px, py, 2, SSD1306_WHITE);
        }
    }
}

void drawSettingsScreen(float selectorY, float yScrollOffset) {
    display.setFont(NULL);
    display.setTextSize(1);

    // Малюємо селектор відносно видимої області
    display.fillRect(0, round(selectorY), display.width(), 10, SSD1306_WHITE);
    
    int16_t x1, y1;
    uint16_t w, h;

    for (int i = 0; i < SETTINGS_ITEM_COUNT; i++) {
        int itemY = i * 11 + 2 + yScrollOffset;
        if (itemY < -8 || itemY > 32) continue;

        bool isSelected = (round(animatedSettingsPosition) == i);
        display.setTextColor(isSelected ? SSD1306_BLACK : SSD1306_WHITE);
        display.setCursor(3, itemY);
        display.print(settingsItems[i]);

        uint16_t elementBgColor = isSelected ? SSD1306_BLACK : SSD1306_WHITE;
        uint16_t elementFgColor = isSelected ? SSD1306_WHITE : SSD1306_BLACK;

        if (i == 0) { // Brightness
            char brStr[8];
            sprintf(brStr, "%d%%", brightness);
            display.getTextBounds(brStr, 0, 0, &x1, &y1, &w, &h);
            display.fillRoundRect(display.width() - w - 8, itemY - 1, w + 6, 8, 2, elementBgColor);
            display.setTextColor(elementFgColor);
            display.setCursor(display.width() - w - 5, itemY);
            display.print(brStr);
        } else if (i == 1) { // PID-Regulator
            display.fillRoundRect(display.width() - 15, itemY - 1, 13, 8, 2, elementBgColor);
             if (pidEnabled) {
                display.drawLine(display.width() - 13, itemY + 3, display.width() - 11, itemY + 5, elementFgColor);
                display.drawLine(display.width() - 11, itemY + 5, display.width() - 8, itemY + 1, elementFgColor);
             }
        } else if (i == 3) { // Sensitivity
            char sensStr[8];
            sprintf(sensStr, "%d Pa", (int)puffPressureThreshold_Pa);
            display.getTextBounds(sensStr, 0, 0, &x1, &y1, &w, &h);
            display.fillRoundRect(display.width() - w - 8, itemY - 1, w + 6, 8, 2, elementBgColor);
            display.setTextColor(elementFgColor);
            display.setCursor(display.width() - w - 5, itemY);
            display.print(sensStr);
        } else if (i == 4) { // Puff Timeout
            char timeStr[8];
            sprintf(timeStr, "%ds", puffTimeout_s);
            display.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);
            display.fillRoundRect(display.width() - w - 8, itemY - 1, w + 6, 8, 2, elementBgColor);
            display.setTextColor(elementFgColor);
            display.setCursor(display.width() - w - 5, itemY);
            display.print(timeStr);
        } else if (i == 5) { // 2D GFX
            const char* animName = gfxAnimNames[currentGFXAnim];
            display.getTextBounds(animName, 0, 0, &x1, &y1, &w, &h);
            display.fillRoundRect(display.width() - w - 8, itemY - 1, w + 6, 8, 2, elementBgColor);
            display.setTextColor(elementFgColor);
            display.setCursor(display.width() - w - 5, itemY);
            display.print(animName);
        } else if (i == 6) { // 3D GFX
            const char* shape = shapeNames[current3DShape];
            display.getTextBounds(shape, 0, 0, &x1, &y1, &w, &h);
            display.fillRoundRect(display.width() - w - 8, itemY - 1, w + 6, 8, 2, elementBgColor);
            display.setTextColor(elementFgColor);
            display.setCursor(display.width() - w - 5, itemY);
            display.print(shape);
            } else if (i == 7) { // LED Effect
            const char* effect = ledEffectNames[currentLEDEffect];
            display.getTextBounds(effect, 0, 0, &x1, &y1, &w, &h);
            display.fillRoundRect(display.width() - w - 8, itemY - 1, w + 6, 8, 2, elementBgColor);
            display.setTextColor(elementFgColor);
            display.setCursor(display.width() - w - 5, itemY);
            display.print(effect);
        
        }
        
    }


}


// --- Адаптовані та нові функції ---
void readBMP180DataAndDetectPuff() {
  if (millis() - lastPressureReadTime < 100) return;
  lastPressureReadTime = millis();
  float currentPressure = bmp.readPressure();
  if (currentPressure == 0) return;

  if (!isPuffing) {
    pressureReadings[pressureReadingIndex] = currentPressure;
    pressureReadingIndex = (pressureReadingIndex + 1) % PRESSURE_READING_COUNT;
    float totalPressure = 0;
    int validReadings = 0;
    for (int i = 0; i < PRESSURE_READING_COUNT; i++) {
      if (pressureReadings[i] != 0) {
        totalPressure += pressureReadings[i];
        validReadings++;
      }
    }
    if (validReadings > 0) baselinePressure_Pa = totalPressure / validReadings;
  }

  switch (puffDetectorState) {
    case PUFF_IDLE:
      if (!isPuffing && currentPressure < (baselinePressure_Pa - puffPressureThreshold_Pa)) {
        puffDetectorState = PUFF_ACTIVE;
        startPuff();
      }
      break;
    case PUFF_ACTIVE:
      if (currentPressure >= (baselinePressure_Pa - (puffPressureThreshold_Pa * 0.5))) {
        puffDetectorState = PUFF_IDLE;
        if (isPuffing) endPuff();
      }
      break;
  }
}


void updateVaporAnimation() {
  unsigned long now = millis();
  if (now - vaporAnimLastUpdate > 10) {
    vaporAnimLastUpdate = now;
    if (isPuffing) {
      if (vaporAnimProgress < 1.0f) vaporAnimProgress += 0.02f;
      if (vaporAnimProgress > 1.0f) vaporAnimProgress = 1.0f;
    } else {
      if (vaporAnimProgress > 0.0f) vaporAnimProgress -= 0.02f;
      if (vaporAnimProgress < 0.0f) vaporAnimProgress = 0.0f;
    }
  }
}

void updateLed() {
    // 1. Пріоритет для Light Painting
    if (isShaking && millis() - lastShakeTime < SHAKE_TIMEOUT) {
        strip.setPixelColor(0, strip.ColorHSV(random(0, 65535)));
        strip.show();
        ledBrightness = 0; // Скидаємо яскравість, щоб наступна затяжка почалась плавно
        return;
    } else {
        isShaking = false; // Ефект закінчився
    }

    // 2. Плавна зміна яскравості
    if (isPuffing && ledBrightness < 1.0f) {
        ledBrightness += 0.01f;
        if (ledBrightness > 1.0f) ledBrightness = 1.0f;
    } else if (!isPuffing && ledBrightness > 0.0f) {
        ledBrightness -= 0.01f;
        if (ledBrightness < 0.0f) ledBrightness = 0.0f;
    }

    // 3. Якщо діод не світиться, вимикаємо і виходимо
    if (ledBrightness <= 0.0f) {
        strip.clear();
        strip.show();
        return;
    }

    // 4. Визначаємо базовий колір згідно з обраним режимом
    uint32_t baseColor = 0;
    switch (currentLEDEffect) {
        case FX_RAINBOW:
            baseColor = strip.ColorHSV(millis() * 10); // Трохи швидше для кращого ефекту
            break;
        case FX_RANDOM_PUFF:
        case FX_STATIC_RED:
        case FX_STATIC_GREEN:
        case FX_STATIC_BLUE:
        case FX_STATIC_PURPLE:
        case FX_STATIC_YELLOW:
        case FX_STATIC_WHITE:
            baseColor = currentPuffColor;
            break;
    }
  
    // 5. Застосовуємо яскравість до кольору
    uint8_t r = (baseColor >> 16) & 0xFF;
    uint8_t g = (baseColor >> 8) & 0xFF;
    uint8_t b = baseColor & 0xFF;
    strip.setPixelColor(0, strip.Color(r * ledBrightness, g * ledBrightness, b * ledBrightness));
    strip.show();
}

void drawVaporAnimation() {
    if (vaporAnimProgress > 0) {
        int vaporWidth = vaporAnimProgress * (display.width() / 2);
        int vaporHeight = vaporAnimProgress * (display.height() - 12);
        display.fillRoundRect((display.width()/2) - vaporWidth, display.height() - vaporHeight -2, vaporWidth*2, vaporHeight, 4, SSD1306_INVERSE);
    }
}// ...existing code...

void handleButtons() {
    unsigned long now = millis();
    for (uint8_t i = 0; i < BTN_COUNT; ++i) {
        bool r = digitalRead(btnPins[i]);
        if (r != btnReading[i]) {
            btnLastChange[i] = now;
            btnReading[i] = r;
        } else if ((now - btnLastChange[i]) > DEBOUNCE_MS && btnReading[i] != btnStable[i]) {
            bool wasPressed = (btnStable[i] == LOW);
            btnStable[i] = btnReading[i];

            if (!wasPressed && btnStable[i] == LOW) { // --- Обробка натискання ---
                bool stateChanged = false;
                
                if (currentState == STATE_MAIN_SCREEN || currentState == STATE_MENU_VIEW) {
                    if (currentState == STATE_MAIN_SCREEN && i == 0) { // Center (M) on Main is Fire
                        startPuff();
                    } else {
                        switch (i) {
                            case 1: if (currentState == STATE_MAIN_SCREEN) wattage = min(200, wattage + 1); break;
                            case 2: if (currentState == STATE_MAIN_SCREEN) wattage = max(5, wattage - 1); break;
                            case 3: animTargetMenuPos++; startMenuAnimation(); break;
                            case 4: animTargetMenuPos--; startMenuAnimation(); break;
                            case 0:
                                if (!isAnimating && ((int)round(animTargetMenuPos) % MAIN_MENU_COUNT + MAIN_MENU_COUNT) % MAIN_MENU_COUNT == 1) {
                                    currentState = STATE_SETTINGS_MENU; stateChanged = true;
                                    currentSettingsItem = 0; animTargetSettingsPos = 0; animatedSettingsPosition = 0;
                                    animTargetSettingsScroll = 0; animatedSettingsScroll = 0;
                                }
                                break;
                        }
                        if (!stateChanged && (i==3 || i==4)) {
                            int targetMenuIndexWrap = ((int)round(animTargetMenuPos) % MAIN_MENU_COUNT + MAIN_MENU_COUNT) % MAIN_MENU_COUNT;
                            currentState = (targetMenuIndexWrap == 0) ? STATE_MAIN_SCREEN : STATE_MENU_VIEW;
                        }
                    }
                } else if (currentState == STATE_SETTINGS_MENU) {
                     switch (i) {
                        case 0: // Center (M)
                            if (!isAnimating) {
                                if (currentSettingsItem == 2) { currentState = STATE_POWER_CURVE; selectedCurvePoint = 0; } 
                                else if (currentSettingsItem == 5) { // 2D GFX
                                    currentState = STATE_GFX_PREVIEW;
                                    isPuffing = true;
                                    clearAllParticles();
                                } else if (currentSettingsItem == 6) { // 3D GFX
                                    currentState = STATE_3D_PREVIEW;
                                    } else if (currentSettingsItem == 7) { // LED Effect
                                } else if (currentSettingsItem == 8) { // Exit
                                    currentState = STATE_MENU_VIEW;
                                }
                            }
                            break;
                        case 1: // UP
                            currentSettingsItem = (currentSettingsItem + SETTINGS_ITEM_COUNT - 1) % SETTINGS_ITEM_COUNT;
                            animTargetSettingsPos = currentSettingsItem;
                            animTargetSettingsScroll = max(0.0f, animTargetSettingsPos - 2.0f);
                            startSettingsAnimation();
                            break;
                        case 2: // DOWN
                            currentSettingsItem = (currentSettingsItem + 1) % SETTINGS_ITEM_COUNT;
                            animTargetSettingsPos = currentSettingsItem;
                            animTargetSettingsScroll = max(0.0f, animTargetSettingsPos - 2.0f);
                            startSettingsAnimation();
                            break;
                        case 3: // Right (R)
                            if (currentSettingsItem == 0) { brightness = min(100, brightness + 10); display.ssd1306_command(SSD1306_SETCONTRAST); display.ssd1306_command(map(brightness, 0, 100, 0, 255)); } 
                            else if (currentSettingsItem == 1) { pidEnabled = !pidEnabled; } 
                            else if (currentSettingsItem == 3) { puffPressureThreshold_Pa = min(100.0f, puffPressureThreshold_Pa + 5.0f); } 
                            else if (currentSettingsItem == 4) { puffTimeout_s = min(10, puffTimeout_s + 1); }
                            else if (currentSettingsItem == 5) { currentGFXAnim = (GFXAnimation)((currentGFXAnim + 1) % GFX_ANIM_COUNT); clearAllParticles(); } 
                            else if (currentSettingsItem == 6) { current3DShape = (ShapeType)((current3DShape + 1) % SHAPE_COUNT); }
                            else if (currentSettingsItem == 7) { currentLEDEffect = (LEDEffect)((currentLEDEffect + 1) % LED_EFFECT_COUNT); }
                            
                            break;
                         case 4: // Left (L)
                            if (currentSettingsItem == 0) { brightness = max(0, brightness - 10); display.ssd1306_command(SSD1306_SETCONTRAST); display.ssd1306_command(map(brightness, 0, 100, 0, 255)); } 
                            else if (currentSettingsItem == 1) { pidEnabled = !pidEnabled; } 
                            else if (currentSettingsItem == 3) { puffPressureThreshold_Pa = max(10.0f, puffPressureThreshold_Pa - 5.0f); } 
                            else if (currentSettingsItem == 4) { puffTimeout_s = max(1, puffTimeout_s - 1); }
                            else if (currentSettingsItem == 5) { currentGFXAnim = (GFXAnimation)((currentGFXAnim + GFX_ANIM_COUNT - 1) % GFX_ANIM_COUNT); clearAllParticles(); } 
                            else if (currentSettingsItem == 6) { current3DShape = (ShapeType)((current3DShape + SHAPE_COUNT - 1) % SHAPE_COUNT); }
                            else if (currentSettingsItem == 7) { currentLEDEffect = (LEDEffect)((currentLEDEffect + LED_EFFECT_COUNT - 1) % LED_EFFECT_COUNT); }
                            
                            break;
                    }
                } else if (currentState == STATE_POWER_CURVE) {
                    // ... (логіка для редактора кривих без змін) ...
                } else if (currentState == STATE_GFX_PREVIEW) {
                    switch (i) {
                        case 3: currentGFXAnim = (GFXAnimation)((currentGFXAnim + 1) % GFX_ANIM_COUNT); clearAllParticles(); break;
                        case 4: currentGFXAnim = (GFXAnimation)((currentGFXAnim + GFX_ANIM_COUNT - 1) % GFX_ANIM_COUNT); clearAllParticles(); break;
                        default: currentState = STATE_SETTINGS_MENU; isPuffing = false; clearAllParticles(); break;
                    }
                } else if (currentState == STATE_3D_PREVIEW) {
                    switch (i) {
                        case 3: current3DShape = (ShapeType)((current3DShape + 1) % SHAPE_COUNT); break;
                        case 4: current3DShape = (ShapeType)((current3DShape + SHAPE_COUNT - 1) % SHAPE_COUNT); break;
                        default: currentState = STATE_SETTINGS_MENU; break;
                    }
                }
            }
            // --- Обробка відпускання кнопки ---
            else if (wasPressed && btnStable[i] == HIGH) {
                if (currentState == STATE_MAIN_SCREEN && i == 0) {
                    endPuff();
                    // Розблокування тепер відбувається в startPuff(), тому звідси його видалено.
                }
            }
        }
    }
}