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

namespace {
constexpr int kWindowSize = 10;
constexpr int kFeatures = 24;
constexpr int kInputElems = kWindowSize * kFeatures;
constexpr int kNumClasses = 8;   // change to your actual class count
constexpr int kTensorArenaSize = 20 * 1024;  // tune this

alignas(16) static uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;

int8_t rolling_window[kInputElems];
int write_frame_idx = 0;
int valid_frames = 0;
}

static bool init_model()
{
    model = tflite::GetModel(gesture_model_int8_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema mismatch: model=%d runtime=%d",
                 model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    static tflite::MicroMutableOpResolver<8> resolver;
    resolver.AddFullyConnected();
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddReshape();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddMean();
    // Adjust to exactly match your model ops.

    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);

    interpreter = &static_interpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed");
        return false;
    }

    input = interpreter->input(0);
    output = interpreter->output(0);

    ESP_LOGI(TAG, "Input type=%d dims=%d [%d,%d,%d]",
             input->type,
             input->dims->size,
             input->dims->data[0],
             input->dims->data[1],
             input->dims->data[2]);

    ESP_LOGI(TAG, "Input quant: scale=%f zp=%d",
             input->params.scale, input->params.zero_point);

    ESP_LOGI(TAG, "Output type=%d count=%d quant: scale=%f zp=%d",
             output->type,
             output->bytes,
             output->params.scale, output->params.zero_point);

    return true;
}