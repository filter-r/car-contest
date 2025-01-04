import sensor, image, time, math, os, tf
from machine import UART
from image import SEARCH_EX
# 初始化传感器和串口
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_brightness(2000)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
clock = time.clock()

uart = UART(2, baudrate=115200)
# 加载模型和标签
net_path_aim = "mobilenet_V5.0.tflite"
labels_aim = [line.rstrip() for line in open("sd/lables.txt")]
net_aim = tf.load(net_path_aim, load_to_fb=True)

net_path_let = "mobilenet_let_V2.0.tflite"
labels_let = [line.rstrip() for line in open("sd/lables_let.txt")]
net_let = tf.load(net_path_let, load_to_fb=True)

net_path_num = "mobilenet_num_V4.0.tflite"
labels_num = [line.rstrip() for line in open("sd/lables_num.txt")]
net_num = tf.load(net_path_num, load_to_fb=True)


def is_square(r, tolerance=0.1):
    w, h = r[2], r[3]
    return abs(w - h) / max(w, h) < tolerance

def model_classification_aim():
    best_result = None
    best_score = 0
    for _ in range(3):
        img = sensor.snapshot().lens_corr(strength=1.3,zoom=1.0)
        for r in img.find_rects(threshold=15000):
            rect = r.rect()
            if isinstance(rect, (tuple, list)) and is_square(rect):
                img1 = img.copy(1, 1, rect)
                for obj in tf.classify(net_aim, img1, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
                    sorted_list = sorted(zip(labels_aim, obj.output()), key=lambda x: x[1], reverse=True)
                    highest_label, highest_score = sorted_list[0]

                    if highest_score > best_score:
                        best_score = highest_score
                        best_result = highest_label

    if best_result is not None:
        x = int(best_result)
        x = "{:03d}".format(x)
        uart.write(b'x' + x)
        print(x)

def model_classification_let():
    best_result = None
    best_score = 0
    for _ in range(3):
        img = sensor.snapshot().lens_corr(strength=1.3,zoom=1.0)
        for r in img.find_rects(threshold=15000):
            rect = r.rect()
            if isinstance(rect, (tuple, list)) and is_square(rect):
                img1 = img.copy(1, 1, rect)
                for obj in tf.classify(net_let, img1, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
                    sorted_list = sorted(zip(labels_let, obj.output()), key=lambda x: x[1], reverse=True)
                    highest_label, highest_score = sorted_list[0]

                    if highest_score > best_score:
                        best_score = highest_score
                        best_result = highest_label

    if best_result is not None:
        x = int(best_result)
        x = "{:03d}".format(x)
        uart.write(b'x' + x)
        print(x)

def model_classification_num():
    best_result = None
    best_score = 0
    for _ in range(3):
        img = sensor.snapshot().lens_corr(strength=1.3,zoom=1.0)
        for r in img.find_rects(threshold=15000):
            rect = r.rect()
            if isinstance(rect, (tuple, list)) and is_square(rect):
                img1 = img.copy(1, 1, rect)
                for obj in tf.classify(net_num, img1, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
                    sorted_list = sorted(zip(labels_num, obj.output()), key=lambda x: x[1], reverse=True)
                    highest_label, highest_score = sorted_list[0]

                    if highest_score > best_score:
                        best_score = highest_score
                        best_result = highest_label

    if best_result is not None:
        x = int(best_result)
        x = "{:03d}".format(x)
        uart.write(b'x' + x)
        print(x)


while True:
    if(uart.read(1) == b'a'):
        rx=1
    elif(uart.read(1) == b'l'):
        rx=2
    elif(uart.read(1) == b'n'):
        rx=3
    if(rx==1):
        model_classification_aim()
    elif(rx==2):
        model_classification_let()
    elif(rx==3):
         model_classification_num()
 # 每次循环后休眠1秒







