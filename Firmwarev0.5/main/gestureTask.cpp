#include <cstring>
#include <cstdint>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "gestureModelData.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char *TAG = "GestureTask";

//MUST MATCH MAIN.C
#define NUMBER_OF_SENSORS 6 

//File specific
#define RESOLVER_CAPACITY 9

// --- Model / input config CHANGE TO MATCH TRAINING IF YOU CHANGE IT
constexpr int kWindowSize   = 10; //Number of Frames
constexpr int kNumImus      = NUMBER_OF_SENSORS; //
constexpr int kFeatures     = kNumImus * 4;   // 24 int8 values from the 6 quaternions
constexpr int kNumClasses   = 5;              // base, gesture1..4 Hand flat, 4 fingers up, OK sign, fist, thumb up/sideways
constexpr int kTensorArenaSize = 20 * 1024;   // idk what this does lol

// Confidence threshold (0.0–1.0)
static float s_min_confidence = 0.90f; // How low we will still accept gestures to be

// QUAT scaling to change them all from int16_t to float
constexpr float kQuatScalingFactor = 16384.0f;

// Sampling / task timing
constexpr TickType_t kGestureTaskPeriodTicks = pdMS_TO_TICKS(100);

// Tensor arena
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];

// TFLM globals
namespace {
const tflite::Model *g_model          = nullptr;
tflite::MicroInterpreter *g_interpreter = nullptr;
TfLiteTensor *g_input                 = nullptr;
TfLiteTensor *g_output                = nullptr;

// Rolling window of float features [time, feature]
static float g_window[kWindowSize][kFeatures];
static int   g_write_frame_idx = 0;
static int   g_valid_frames    = 0;
}  // namespace

// --- Labels (must match encoder.classes_ order) ---
static const char *kClassNames[kNumClasses] = {
    "base",
    "gesture1",
    "gesture2",
    "gesture3",
    "gesture4"
};

// --- Sensor data source ---
// From main.c (or wherever you define it):
// int16_t printableQuat[NUMBER_OF_SENSORS][4];
// Here we just declare it as extern with 6 sensors.
extern "C" {
    extern int16_t printableQuat[NUMBER_OF_SENSORS][4];
}

// --------------------------------------------------
//  TFLite Micro initialization
// --------------------------------------------------
static bool InitModel()
{
    g_model = tflite::GetModel(gesture_model_int8_tflite);
    if (g_model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema mismatch: model=%ld runtime=%d",
                 g_model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    static tflite::MicroMutableOpResolver<RESOLVER_CAPACITY> resolver;
    resolver.AddExpandDims();
    resolver.AddConv2D();
    resolver.AddMul();
    resolver.AddAdd();
    resolver.AddReshape();
    resolver.AddMaxPool2D();
    resolver.AddMean();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();

    static tflite::MicroInterpreter static_interpreter(
        g_model, resolver, tensor_arena, kTensorArenaSize);

    g_interpreter = &static_interpreter;

    if (g_interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed");
        return false;
    }

    g_input  = g_interpreter->input(0);
    g_output = g_interpreter->output(0);

    ESP_LOGI(TAG, "Input: type=%d dims=%d [%d,%d,%d]",
             g_input->type,
             g_input->dims->size,
             g_input->dims->data[0],
             g_input->dims->data[1],
             g_input->dims->data[2]);

    ESP_LOGI(TAG, "Input quant: scale=%f zp=%ld",
             g_input->params.scale, g_input->params.zero_point);

    ESP_LOGI(TAG, "Output: type=%d bytes=%d quant: scale=%f zp=%ld",
             g_output->type,
             g_output->bytes,
             g_output->params.scale, g_output->params.zero_point);

    return true;
}

// --------------------------------------------------
//  Window management
// --------------------------------------------------
static void PushFrameFromPrintableQuat()
{
    // Convert current printableQuat (int16 scaled by 16384) into
    // a single frame of 24 floats in [-1, 1].
    float *frame = g_window[g_write_frame_idx];

    int idx = 0;
    for (int imu = 0; imu < kNumImus; ++imu) {
        for (int c = 0; c < 4; ++c) {
            int16_t raw = printableQuat[imu][c];
            frame[idx++] = static_cast<float>(raw) / kQuatScalingFactor;
        }
    }

    g_write_frame_idx = (g_write_frame_idx + 1) % kWindowSize;
    if (g_valid_frames < kWindowSize) {
        g_valid_frames++;
    }
}

static bool WindowIsFull()
{
    return g_valid_frames >= kWindowSize;
}

// --------------------------------------------------
//  Inference
// --------------------------------------------------
static bool RunInference(const char *&out_label, float &out_confidence)
{
    if (!WindowIsFull()) {
        return false;
    }

    // Quantize window into input tensor
    if (g_input->type != kTfLiteInt8) {
        ESP_LOGE(TAG, "Expected int8 input tensor");
        return false;
    }

    int8_t *input_data = g_input->data.int8;
    const float scale  = g_input->params.scale;
    const int   zp     = g_input->params.zero_point;

    int idx = 0;
    for (int t = 0; t < kWindowSize; ++t) {
        for (int f = 0; f < kFeatures; ++f) {
            float v = g_window[(g_write_frame_idx + t) % kWindowSize][f];
            int32_t q = static_cast<int32_t>(std::round(v / scale)) + zp;
            if (q < -128) q = -128;
            if (q > 127)  q = 127;
            input_data[idx++] = static_cast<int8_t>(q);
        }
    }

    if (g_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed");
        return false;
    }

    // Dequantize output and find argmax
    if (g_output->type != kTfLiteInt8) {
        ESP_LOGE(TAG, "Expected int8 output tensor");
        return false;
    }

    const int8_t *out_data = g_output->data.int8;
    const float out_scale  = g_output->params.scale;
    const int   out_zp     = g_output->params.zero_point;

    int   best_idx   = 0;
    float best_prob  = -1.0f;

    for (int i = 0; i < kNumClasses; ++i) {
        float prob = (static_cast<int32_t>(out_data[i]) - out_zp) * out_scale;
        if (prob > best_prob) {
            best_prob = prob;
            best_idx  = i;
        }
    }

    out_label      = kClassNames[best_idx];
    out_confidence = best_prob;  // already in [0,1] if softmax

    return true;
}

// --------------------------------------------------
//  Implementation fo the
// --------------------------------------------------
extern "C" void classifyGesture_Task(void *pvParameters)
{
    ESP_LOGI(TAG, "Gesture task starting...");

    if (!InitModel()) {
        ESP_LOGE(TAG, "Model init failed, killing task");
        vTaskDelete(nullptr);
        return;
    }

    while (true) {
        // 1) Capture current frame from printableQuat
        PushFrameFromPrintableQuat();

        // 2) Run inference when window is full
        if (WindowIsFull()) {
            const char *label = nullptr;
            float confidence  = 0.0f;

            if (RunInference(label, confidence)) {
                if (confidence < s_min_confidence) {
                    printf("Gesture: %-10s  (raw: %s @ %.1f%%)\n",
                           "uncertain", label, confidence * 100.0f);
                } else {
                    printf("Gesture: %-10s  Confidence: %.1f%%\n",
                           label, confidence * 100.0f);
                }
                fflush(stdout);
            }
        }

        vTaskDelay(kGestureTaskPeriodTicks);  // ~100 ms
    }
}
