import os
import cv2
import sys
import argparse
import time
import numpy as np
import math

# add path
# realpath = os.path.abspath(__file__)
# _sep = os.path.sep
# realpath = realpath.split(_sep)
# sys.path.append(os.path.join(realpath[0]+_sep, *realpath[1:realpath.index('rknn_model_zoo')+1]))

from .coco_utils import COCO_test_helper
from rknn.api import RKNN  # Прямой импорт RKNN

OBJ_THRESH = 0.25
NMS_THRESH = 0.45
IMG_SIZE = (640, 640)

# CLASSES = ("person", "bicycle", "car","motorbike ","aeroplane ","bus ","train","truck ","boat","traffic light",
#            "fire hydrant","stop sign ","parking meter","bench","bird","cat","dog ","horse ","sheep","cow","elephant",
#            "bear","zebra ","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite",
#            "baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife ",
#            "spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza ","donut","cake","chair","sofa",
#            "pottedplant","bed","diningtable","toilet ","tvmonitor","laptop	","mouse	","remote ","keyboard ","cell phone","microwave ",
#            "oven ","toaster","sink","refrigerator ","book","clock","vase","scissors ","teddy bear ","hair drier", "toothbrush ")

CLASSES = (
    "человек", "велосипед", "автомобиль", "мотоцикл", "самолет", "автобус", "поезд", "грузовик", "лодка", "светофор",
    "пожарный гидрант", "знак остановки", "парковочный счетчик", "скамейка", "птица", "кошка", "собака", "лошадь", "овца", "корова", "слон",
    "медведь", "зебра", "жираф", "рюкзак", "зонт", "сумка", "галстук", "чемодан", "фрисби", "лыжи", "сноуборд", "спортивный мяч", "воздушный змей",
    "бейсбольная бита", "бейсбольная перчатка", "скейтборд", "доска для серфинга", "теннисная ракетка", "бутылка", "бокал", "чашка", "вилка", "нож",
    "ложка", "миска", "банан", "яблоко", "сэндвич", "апельсин", "брокколи", "морковь", "хот-дог", "пицца", "пончик", "торт", "стул", "диван",
    "растение в горшке", "кровать", "обеденный стол", "унитаз", "телевизор", "ноутбук", "мышь", "пульт", "клавиатура", "мобильный телефон", "микроволновая печь",
    "духовка", "тостер", "раковина", "холодильник", "книга", "часы", "ваза", "ножницы", "плюшевый мишка", "фен", "зубная щетка"
)



def calculate_distance(box, focal_length, real_height=1.7):
    """
    Calculate distance to object using bbox height and camera calibration
    box: [x1, y1, x2, y2] in pixels
    focal_length: focal length in pixels from camera matrix
    real_height: real height of object in meters (default 1.7m for person)
    """
    # Calculate bbox height in pixels
    bbox_height = box[3] - box[1]  # bottom - top
    
    if bbox_height <= 0:
        return None
    
    # Distance = (real_height * focal_length) / bbox_height
    distance = (real_height * focal_length) / bbox_height
    return distance

def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with object threshold."""
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score* box_confidences >= OBJ_THRESH)
    scores = (class_max_score* box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores

def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes."""
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep

def dfl(position):
    # Distribution Focal Loss (DFL)
    import torch
    x = torch.tensor(position)
    n,c,h,w = x.shape
    p_num = 4
    mc = c//p_num
    y = x.reshape(n,p_num,mc,h,w)
    y = y.softmax(2)
    acc_metrix = torch.tensor(range(mc)).float().reshape(1,1,mc,1,1)
    y = (y*acc_metrix).sum(2)
    return y.numpy()

def post_process(outputs, conf_thres=OBJ_THRESH, iou_thres=NMS_THRESH):

    if len(outputs) < 1:
        return None, None, None

    # Первый выход: (1, 116, N) или (1, N, 116)
    dets = outputs[0].squeeze()  # (116, N) или (N, 116)

    # Приводим к форме (N, 116)
    if dets.shape[0] == 116:
        dets = dets.T  # (N, 116)

    # 4 bbox + num_classes + 32 mask_coeffs
    total_features = dets.shape[1]
    num_classes = total_features - 4 - 32  # 4 bbox + 32 маски

    # Разбираем
    boxes_xywh = dets[:, :4]                              # (N,4)
    class_logits = dets[:, 4:4 + num_classes]             # (N,80)
    mask_coeffs = dets[:, 4 + num_classes:]               # (N,32) — сейчас не используем

    # Максимальная уверенность по классам
    scores = np.max(class_logits, axis=1)
    class_ids = np.argmax(class_logits, axis=1)

    # Фильтр по порогу
    keep = scores >= conf_thres
    boxes_xywh = boxes_xywh[keep]
    scores = scores[keep]
    class_ids = class_ids[keep]

    if len(boxes_xywh) == 0:
        return None, None, None

    # Преобразуем xywh (нормализованные) в xyxy в сетке 640x640
    # По аналогии с твоим segmentation_node.py
    x = boxes_xywh[:, 0]
    y = boxes_xywh[:, 1]
    w = boxes_xywh[:, 2]
    h = boxes_xywh[:, 3]

    x1 = x - w / 2.0
    y1 = y - h / 2.0
    x2 = x + w / 2.0
    y2 = y + h / 2.0

    boxes_640 = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)

    # NMS через OpenCV
    boxes_nms = np.hstack(
        [boxes_640[:, :2], boxes_640[:, 2:] - boxes_640[:, :2]]
    )  # (x,y,w,h)

    indices = cv2.dnn.NMSBoxes(
        bboxes=boxes_nms.tolist(),
        scores=scores.tolist(),
        score_threshold=conf_thres,
        nms_threshold=iou_thres,
    )
    if len(indices) == 0:
        return None, None, None

    indices = indices.flatten()

    boxes_640 = boxes_640[indices]
    scores = scores[indices]
    class_ids = class_ids[indices]

    # Возвращаем в формате, который ожидает draw():
    # (top, left, right, bottom) = (x1, y1, x2, y2)
    boxes = boxes_640.astype(np.int32)

    return boxes, class_ids, scores

def is_valid_person_aspect_ratio(box, min_ratio=2.5, max_ratio=5.0):
    """
    Check if person bbox has valid aspect ratio for distance calculation
    Typical standing person has height/width ratio around 2.5-3.5
    """
    height = box[3] - box[1]  # bottom - top
    width = box[2] - box[0]   # right - left
    
    if width <= 0:
        return False
    
    aspect_ratio = height / width
    return min_ratio <= aspect_ratio <= max_ratio

def draw(image, boxes, scores, classes, focal_length=594.9):
    list_object = []
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = [int(_b) for _b in box]
        # Ensure coordinates are within image bounds
        top = max(0, min(top, image.shape[1]))
        left = max(0, min(left, image.shape[0]))
        right = max(0, min(right, image.shape[1]))
        bottom = max(0, min(bottom, image.shape[0]))
        
        class_name = CLASSES[cl] if cl < len(CLASSES) else f"Class_{cl}"
        distance = None
        # Set color: green for person, blue for others
        if class_name == "человек":
            color = (0, 255, 0)  # Green
            
            # Check if person has valid aspect ratio for distance calculation
            is_valid_ratio = is_valid_person_aspect_ratio([top, left, right, bottom])
            
            if is_valid_ratio:
                # Calculate distance for person with valid aspect ratio
                distance = calculate_distance([top, left, right, bottom], focal_length)
                if distance is not None:
                    distance_text = f"{distance:.1f}m"
                    status = "VALID"
                    print(f"Person @ ({top} {left} {right} {bottom}) {score:.3f} - Distance: {distance:.1f}m [VALID ASPECT RATIO]")
                else:
                    distance_text = "N/A"
                    status = "INVALID"
            else:
                # Person detected but aspect ratio invalid for distance calculation
                distance_text = "unknown"
                status = "INVALID_ASPECT"
                aspect_ratio = (bottom - left) / (right - top) if (right - top) > 0 else 0
                print(f"Person @ ({top} {left} {right} {bottom}) {score:.3f} - Distance: unknown [INVALID ASPECT RATIO: {aspect_ratio:.2f}]")
        else:
            color = (255, 0, 0)  # Blue
            distance_text = ""
            status = "OTHER"
            print(f"{class_name} @ ({top} {left} {right} {bottom}) {score:.3f}")
        color = get_color_for_class(class_name)
        color = [int(i * 255) for i in color]
        # Draw bounding box
        cv2.rectangle(image, (top, left), (right, bottom), color, 2)
        list_object.append([(top, left, right, bottom), distance, class_name])
        # Prepare label - different format based on status
        if status == "VALID":
            label = f'Person {score:.2f} {distance_text}'
        elif status == "INVALID_ASPECT":
            label = f'Person {score:.2f} (unknown dist)'
        else:
            label = f'{class_name} {score:.2f}'
        
        # Draw label background
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_COMPLEX, 0.6, 2)[0]
        cv2.rectangle(image, (top, left - label_size[1] - 10), 
                     (top + label_size[0], left), color, -1)
        # Draw label text
        cv2.putText(image, label,
                    (top, left - 6), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 2)
    return list_object

def setup_rknn_model(model_path, core_mask=7):
    """Инициализация RKNN модели с поддержкой multi-core"""
    rknn = RKNN()
    
    print('--> Loading RKNN model')
    ret = rknn.load_rknn(model_path)
    if ret != 0:
        print('Load RKNN model failed!')
        exit(ret)
    
    print('--> Init runtime')
    
    print(f'Using NPU cores: {core_mask}')
    
    ret = rknn.init_runtime(
        target='rk3588',
        device_id=None,
        core_mask=core_mask,  # Вот где указываются ядра!
        perf_debug=True,
        eval_mem=True
    )
    
    if ret != 0:
        print('Init runtime failed!')
        exit(ret)
    
    print('✅ RKNN model loaded successfully!')
    return rknn

def load_camera_calibration(calib_path='camera_calibration.txt'):
    """Load camera calibration parameters from file"""
    try:
        with open(calib_path, 'r') as f:
            lines = f.readlines()
        
        focal_length = None
        for i, line in enumerate(lines):
            if 'Матрица камеры:' in line and i + 1 < len(lines):
                # Parse camera matrix line: 594.900106 0.000000 315.535008
                matrix_line = lines[i + 1].strip()
                fx = float(matrix_line.split()[0])  # focal length x
                focal_length = fx
                break
        
        return focal_length if focal_length else 594.9  # fallback to your camera's focal length
    
    except Exception as e:
        print(f"Warning: Could not load camera calibration: {e}")
        print("Using default focal length: 594.9")
        return 594.9  # fallback to your camera's focal length

def get_color_for_class(class_name):
    """
    Возвращает цвет для класса объекта на русском языке
    """
    color_map = {
        # Люди и животные - теплые цвета
        "человек": (1.0, 0.0, 0.0),        # красный
        "птица": (1.0, 0.5, 0.0),          # оранжевый
        "кошка": (1.0, 0.0, 0.5),          # розовый
        "собака": (0.8, 0.2, 0.2),         # светло-красный
        "лошадь": (0.7, 0.3, 0.0),         # коричнево-оранжевый
        "овца": (0.9, 0.9, 0.9),           # белый
        "корова": (0.4, 0.2, 0.0),         # коричневый
        "слон": (0.3, 0.3, 0.3),           # темно-серый
        "медведь": (0.2, 0.1, 0.0),        # темно-коричневый
        "зебра": (0.1, 0.1, 0.1),          # черный
        "жираф": (1.0, 0.8, 0.2),          # желтый
        
        # Транспорт - синие и зеленые тона
        "велосипед": (0.0, 1.0, 0.0),      # зеленый
        "автомобиль": (0.0, 0.0, 1.0),     # синий
        "мотоцикл": (0.0, 0.5, 1.0),       # голубой
        "самолет": (0.2, 0.6, 1.0),        # небесно-голубой
        "автобус": (0.0, 0.0, 0.8),        # темно-синий
        "поезд": (0.0, 0.3, 0.6),          # сине-зеленый
        "грузовик": (0.0, 0.2, 0.5),       # морской волны
        "лодка": (0.0, 0.4, 0.8),          # аквамариновый
        
        # Дорожная инфраструктура - желтые и красные
        "светофор": (1.0, 0.0, 0.0),       # красный
        "пожарный гидрант": (1.0, 0.0, 0.0), # красный
        "знак остановки": (1.0, 0.0, 0.0), # красный
        "парковочный счетчик": (1.0, 1.0, 0.0), # желтый
        
        # Мебель и интерьер - коричневые тона
        "скамейка": (0.4, 0.2, 0.0),       # коричневый
        "стул": (0.5, 0.3, 0.1),           # светло-коричневый
        "диван": (0.6, 0.4, 0.2),          # бежевый
        "кровать": (0.7, 0.5, 0.3),        # песочный
        "обеденный стол": (0.4, 0.2, 0.1), # темно-коричневый
        
        # Электроника - фиолетовые и синие
        "телевизор": (0.5, 0.0, 0.5),      # фиолетовый
        "ноутбук": (0.6, 0.0, 0.6),        # пурпурный
        "мышь": (0.7, 0.0, 0.7),           # светло-фиолетовый
        "пульт": (0.8, 0.0, 0.8),          # розово-фиолетовый
        "клавиатура": (0.9, 0.0, 0.9),     # ярко-фиолетовый
        "мобильный телефон": (0.5, 0.2, 0.8), # лавандовый
        "микроволновая печь": (0.6, 0.3, 0.9), # сиреневый
        "духовка": (0.4, 0.1, 0.7),         # темно-фиолетовый
        "тостер": (0.7, 0.4, 1.0),          # светло-сиреневый
        "холодильник": (0.8, 0.9, 1.0),     # голубовато-белый
        
        # Кухонные предметы - оранжевые и желтые
        "бутылка": (0.0, 0.8, 0.8),        # бирюзовый
        "бокал": (0.8, 0.9, 1.0),          # прозрачно-голубой
        "чашка": (0.9, 0.6, 0.3),          # терракотовый
        "вилка": (0.7, 0.7, 0.7),          # серебряный
        "нож": (0.5, 0.5, 0.5),            # стальной
        "ложка": (0.9, 0.9, 0.9),          # металлический
        "миска": (0.8, 0.5, 0.2),          # глиняный
        "раковина": (0.8, 0.8, 0.9),       # бело-голубой
        
        # Еда - аппетитные цвета
        "банан": (1.0, 1.0, 0.0),          # желтый
        "яблоко": (1.0, 0.0, 0.0),         # красный
        "сэндвич": (0.8, 0.6, 0.4),        # коричневый
        "апельсин": (1.0, 0.5, 0.0),       # оранжевый
        "брокколи": (0.0, 0.5, 0.0),       # зеленый
        "морковь": (1.0, 0.6, 0.0),        # оранжевый
        "хот-дог": (0.8, 0.4, 0.2),        # коричнево-красный
        "пицца": (0.9, 0.7, 0.3),          # золотистый
        "пончик": (0.6, 0.4, 0.2),         # коричневый
        "торт": (1.0, 0.8, 0.9),           # розовый
        
        # Спорт и развлечения - яркие цвета
        "фрисби": (0.0, 1.0, 1.0),         # бирюзовый
        "лыжи": (1.0, 1.0, 1.0),           # белый
        "сноуборд": (0.8, 0.9, 1.0),       # светло-голубой
        "спортивный мяч": (0.0, 0.5, 0.0), # зеленый
        "воздушный змей": (1.0, 0.0, 1.0), # малиновый
        "бейсбольная бита": (0.4, 0.2, 0.0), # коричневый
        "бейсбольная перчатка": (0.6, 0.3, 0.1), # светло-коричневый
        "скейтборд": (0.2, 0.2, 0.2),      # черный
        "доска для серфинга": (0.0, 0.8, 0.8), # бирюзовый
        "теннисная ракетка": (0.9, 0.9, 0.9), # белый
        
        # Личные вещи - разноцветные
        "рюкзак": (0.2, 0.2, 0.6),         # темно-синий
        "зонт": (0.5, 0.0, 0.5),           # фиолетовый
        "сумка": (0.6, 0.4, 0.2),          # коричневый
        "галстук": (0.1, 0.1, 0.1),        # черный
        "чемодан": (0.3, 0.2, 0.1),        # темно-коричневый
        "плюшевый мишка": (0.8, 0.6, 0.4), # бежевый
        
        # Другие объекты
        "растение в горшке": (0.0, 0.6, 0.0), # зеленый
        "унитаз": (0.9, 0.9, 0.9),         # белый
        "книга": (0.6, 0.4, 0.2),          # коричневый
        "часы": (0.7, 0.7, 0.7),           # серебряный
        "ваза": (0.8, 0.8, 0.9),           # голубовато-белый
        "ножницы": (0.5, 0.5, 0.5),        # серый
        "фен": (0.8, 0.8, 0.8),            # светло-серый
        "зубная щетка": (1.0, 1.0, 1.0),   # белый
    }
    
    return color_map.get(class_name, (0.5, 0.5, 0.5))  # серый по умолчанию для неизвестных классов

def main():
    parser = argparse.ArgumentParser(description='YOLOv8 Real-time Video Detection with Multi-NPU')
    parser.add_argument('--model_path', type=str, default='/home/ubuntu24/ros2_ws/src/security_system/security_system/yolov8n-seg-rk3588.rknn', help='RKNN model path')
    parser.add_argument('--target', type=str, default='rk3588', help='target RKNPU platform')
    parser.add_argument('--device_id', type=str, default=None, help='device id')
    parser.add_argument('--core_mask', type=int, default=7, help='NPU core mask: 1=core0, 2=core1, 4=core2, 7=all cores')
    parser.add_argument('--camera_id', type=int, default=0, help='Camera device ID')
    parser.add_argument('--fps', type=int, default=30, help='Target FPS for display')
    parser.add_argument('--perf_debug', action='store_true', help='Enable performance debug')
    parser.add_argument('--eval_mem', action='store_true', help='Evaluate memory usage')
    parser.add_argument('--calib_path', type=str, default='camera_calibration.txt', help='Camera calibration file path')
    parser.add_argument('--person_height', type=float, default=1.7, help='Average person height in meters')
    
    args = parser.parse_args()

    # Load camera calibration
    focal_length = load_camera_calibration(args.calib_path)
    print(f"Loaded focal length: {focal_length:.2f} pixels")
    print(f"Using person height: {args.person_height} meters")

    # Initialize RKNN model
    rknn = setup_rknn_model(args)
    co_helper = COCO_test_helper(enable_letter_box=True)

    # Initialize camera
    cap = cv2.VideoCapture(args.camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        rknn.release()
        return

    print(f"Video capture started. NPU cores: {args.core_mask}")
    print("Press 'q' to quit, 's' to save frame, '1'-'7' to change core mask")
    
    frame_count = 0
    start_time = time.time()
    current_core_mask = args.core_mask
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame")
                break

            # Preprocess frame for YOLO
            img_src = frame.copy()
            
            # Letterbox resize
            pad_color = (0, 0, 0)
            img = co_helper.letter_box(im=img_src.copy(), new_shape=(IMG_SIZE[1], IMG_SIZE[0]), pad_color=pad_color)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # Run inference
            outputs = rknn.inference(inputs=[img])
            
            boxes, classes, scores = post_process(outputs)

            # Draw results on original frame
            if boxes is not None:
                draw(img_src, co_helper.get_real_box(boxes), scores, classes, focal_length)

            # Calculate and display FPS
            frame_count += 1
            if frame_count >= 30:
                end_time = time.time()
                fps = frame_count / (end_time - start_time)
                print(f"FPS: {fps:.2f}, Core mask: {current_core_mask} ({bin(current_core_mask)})")
                frame_count = 0
                start_time = time.time()
            
            # Display info on frame
            fps_text = f"FPS: {fps:.1f}" if 'fps' in locals() else "FPS: Calculating..."
            cv2.putText(img_src, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img_src, f"Cores: {current_core_mask}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Display frame
            cv2.imshow("YOLOv8 Multi-NPU Detection", img_src)

            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                timestamp = int(time.time())
                filename = f"detection_{timestamp}.jpg"
                cv2.imwrite(filename, img_src)
                print(f"Frame saved as {filename}")
            elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6'), ord('7')]:
                # Dynamic core mask switching
                core_masks = {
                    ord('1'): 1, ord('2'): 2, ord('3'): 3, 
                    ord('4'): 4, ord('5'): 5, ord('6'): 6, ord('7'): 7
                }
                new_mask = core_masks[key]
                if new_mask != current_core_mask:
                    print(f"Switching core mask from {current_core_mask} to {new_mask}...")
                    rknn.release()
                    args.core_mask = new_mask
                    rknn = setup_rknn_model(args)
                    current_core_mask = new_mask

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        rknn.release()
        print("Resources released")

if __name__ == '__main__':
    main()
