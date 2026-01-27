import os
from PIL import Image, ImageDraw, ImageEnhance
import cv2
import random
import h5py
import numpy as np
import shutil

def adjust_lightness(image, lightness):
    if lightness != 0:
        enhancer = ImageEnhance.Brightness(image)
        return enhancer.enhance(1 + lightness / 100)
    return image

def draw_rectangles(image, rect_params):
    if rect_params:
        draw = ImageDraw.Draw(image)
        for rect, fill in rect_params:
            draw.rectangle(rect, fill=fill)
    return image

def generate_rect_params(rectangles_config, img_width, img_height):
    rect_params = []
    count = rectangles_config.get('count', 0)
    if count > 0:
        for _ in range(count):
            rect_width = random.randint(int(img_width * 0.1), int(img_width * 0.3))
            rect_height = random.randint(int(img_height * 0.1), int(img_height * 0.3))
            x0 = random.randint(0, img_width - rect_width)
            y0 = random.randint(0, img_height - rect_height)
            x1 = x0 + rect_width
            y1 = y0 + rect_height
            
            if rectangles_config.get('randomColor'):
                color_rgb = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            else:
                hex_color = rectangles_config.get('color', '#000000').lstrip('#')
                color_rgb = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
            
                        # color = color_rgb # 기존 코드
            color_bgr = (color_rgb[2], color_rgb[1], color_rgb[0])
            rect_params.append(((x0, y0, x1, y1), color_bgr))
    return rect_params

def add_salt_and_pepper_noise(image, amount):
    if amount > 0:
        output = np.copy(np.array(image))
        # Salt mode
        num_salt = np.ceil(amount * output.size * 0.5)
        coords = [np.random.randint(0, i - 1, int(num_salt)) for i in output.shape]
        output[coords[0], coords[1], :] = 255

        # Pepper mode
        num_pepper = np.ceil(amount * output.size * 0.5)
        coords = [np.random.randint(0, i - 1, int(num_pepper)) for i in output.shape]
        output[coords[0], coords[1], :] = 0
        return Image.fromarray(output)
    return image

def add_gaussian_noise(image, mean, sigma):
    if sigma > 0:
        img_array = np.array(image)
        gaussian_noise = np.random.normal(mean, sigma, img_array.shape)
        noisy_image_array = img_array + gaussian_noise
        noisy_image_array = np.clip(noisy_image_array, 0, 255)
        return Image.fromarray(noisy_image_array.astype('uint8'))
    return image

def generate_prospective_transform(width, height, scale_factor=0, degrees=0, shear=0, perspective=0):
    # 1. Center matrix
    C = np.eye(3)
    C[0, 2] = -width / 2
    C[1, 2] = -height / 2

    perspective = perspective * 0.0001

    # 2. Perspective matrix
    P = np.eye(3)
    P[2, 0] = random.uniform(-perspective, perspective)
    P[2, 1] = random.uniform(-perspective, perspective)

    # 3. Rotation & Scale matrix
    R = np.eye(3)
    a = random.uniform(-degrees, degrees)
    s = random.uniform(1-scale_factor*0.01, 1+scale_factor*0.01)
    R[:2] = cv2.getRotationMatrix2D(angle=a, center=(0, 0), scale=s)

    # 4. Shear matrix
    S = np.eye(3)
    S[0, 1] = np.tan(random.uniform(-shear, shear) * np.pi / 180)
    S[1, 0] = np.tan(random.uniform(-shear, shear) * np.pi / 180)

    # 5. Translation matrix
    T = np.eye(3)
    T[0, 2] = width / 2
    T[1, 2] = height / 2

    # Combined transformation matrix
    M = T @ S @ R @ P @ C 
    return M

def prospective_transform(image, M):
    """
    YOLO 스타일의 Perspective Transform Augmentation
    :param image: 입력 이미지 (PIL Image 객체)
    :param M: 변환 행렬
    :return: 변환된 PIL Image 객체
    """
    if M is None:
        return image
    
    # 1. PIL 이미지 크기 가져오기 및 NumPy 배열 변환
    width, height = image.size
    img_np = np.array(image) # OpenCV 연산을 위해 NumPy 배열로 변환

    # 2. 이미지에 변환 적용 (입력은 NumPy 배열인 img_np)
    result_np = cv2.warpPerspective(img_np, M, dsize=(width, height), borderValue=(114, 114, 114))

    # 3. 다시 PIL 이미지로 변환하여 반환
    return Image.fromarray(result_np)

def apply_hsv(image, rand_h, rand_s, rand_v):
    if rand_h == 0 and rand_s == 1 and rand_v == 1:
        return image
    
    img_np = np.array(image)
    hsv = cv2.cvtColor(img_np, cv2.COLOR_RGB2HSV)
    h, s, v = cv2.split(hsv)
    
    h = h.astype(np.float32)
    h_new = (h + rand_h) % 180
    h_new = h_new.astype(np.uint8)
    
    s = s.astype(np.float32)
    s_new = np.clip(s * rand_s, 0, 255).astype(np.uint8)
    
    v = v.astype(np.float32)
    v_new = np.clip(v * rand_v, 0, 255).astype(np.uint8)
    
    final_hsv = cv2.merge((h_new, s_new, v_new))
    img_rgb = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2RGB)
    
    return Image.fromarray(img_rgb)

def augment_dataset(dataset_id, aug_dataset_id, lightness, rectangles, salt_and_pepper, gaussian, prospective, hsv, socketio_instance, task_control):
    dataset_path = os.path.join('/root/src/backend/datasets', str(dataset_id))
    aug_dataset_path = os.path.join('/root/src/backend/datasets', str(aug_dataset_id))

    if not os.path.exists(dataset_path):
        log_message = f"[ERROR] Dataset path {dataset_path} does not exist."
        print(log_message)
        return

    if os.path.exists(aug_dataset_path):
        shutil.rmtree(aug_dataset_path)
    os.makedirs(aug_dataset_path)

    hdf5_files = [f for f in os.listdir(dataset_path) if f.endswith('.hdf5')]

    socketio_instance.emit('augmentation_progress', {
        'progress': 0,
    })

    h_gain = 0.5
    s_gain = 0.7
    v_gain = 0.4

    for i, hdf5_file in enumerate(hdf5_files):
        if task_control.get('stop'):
            print("Stopping Data Augmentation")
            return

        src_file_path = os.path.join(dataset_path, hdf5_file)
        dest_file_path = os.path.join(aug_dataset_path, hdf5_file)

        try:
            shutil.copy(src_file_path, dest_file_path)

            with h5py.File(dest_file_path, 'a') as f:
                # Determine rectangle parameters for this episode (HDF5 file)
                rect_params = []
                image_keys = list(f['observations/images'].keys())
                
                if hsv:
                    rand_h = (np.random.rand() * 2 - 1) * h_gain * 180
                    rand_s = (np.random.rand() * 2 - 1) * s_gain + 1
                    rand_v = (np.random.rand() * 2 - 1) * v_gain + 1
                else:
                    rand_h, rand_s, rand_v = 0, 1, 1

                if image_keys:
                    first_image_dataset = f['observations/images'][image_keys[0]]
                    if first_image_dataset.shape[0] > 0:
                        img_height, img_width, _ = first_image_dataset.shape[1:]
                        rect_params = generate_rect_params(rectangles, img_width, img_height)
                        transform_matrix = generate_prospective_transform(img_width, img_height, prospective.get('scale_factor', 0), prospective.get('degrees', 0), prospective.get('shear', 0), prospective.get('perspective', 0))

                # Now modify images
                image_group = f['observations/images']
                for key in image_group.keys():
                    images_data = image_group[key][:]
                    
                    if images_data.shape[0] == 0:
                        continue

                    augmented_images = []
                    for img_array in images_data:
                        img = Image.fromarray(img_array)
                        img = adjust_lightness(img, lightness)
                        img = draw_rectangles(img, rect_params)
                        img = add_salt_and_pepper_noise(img, salt_and_pepper.get('amount', 0))
                        img = add_gaussian_noise(img, gaussian.get('mean', 0), gaussian.get('sigma', 0))
                        img = prospective_transform(img, transform_matrix)
                        img = apply_hsv(img, rand_h, rand_s, rand_v)
                        augmented_images.append(np.array(img))
                    
                    image_group[key][...] = np.array(augmented_images)

            socketio_instance.emit('augmentation_progress', {
                'progress': (i+1) / len(hdf5_files),
            })
        except Exception as e:
            log_message = f"[ERROR] Error processing file {hdf5_file}: {e}"
            print(log_message)
            
    socketio_instance.emit('augmentation_complete', {'dataset_id': aug_dataset_id})
    
    task_control['stop'] = True
    return