import tensorflow as tf
import sys

model = tf.lite.Interpreter(model_path="gesture_model_int8.tflite")
model.allocate_tensors()

ops = model._get_ops_details()
for op in ops:
    print(op["op_name"])
