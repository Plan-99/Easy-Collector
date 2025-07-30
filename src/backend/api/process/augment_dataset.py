import os
from PIL import Image, ImageDraw, ImageEnhance
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

def augment_dataset(dataset_id, aug_dataset_id, lightness, rectangles, salt_and_pepper, gaussian, socketio_instance, task_control):
    dataset_path = os.path.join('/root/src/backend/datasets', str(dataset_id))
    aug_dataset_path = os.path.join('/root/src/backend/datasets', str(aug_dataset_id))

    if not os.path.exists(dataset_path):
        log_message = f"Dataset path {dataset_path} does not exist."
        print(log_message)
        socketio_instance.emit('log_augment_dataset', {'log': log_message, 'type': 'stderr'})
        return

    if os.path.exists(aug_dataset_path):
        shutil.rmtree(aug_dataset_path)
    os.makedirs(aug_dataset_path)

    hdf5_files = [f for f in os.listdir(dataset_path) if f.endswith('.hdf5')]

    for hdf5_file in hdf5_files:
        if task_control.get('stop'):
            socketio_instance.emit('log_augment_dataset', {'log': 'Stopping Data Augmentation', 'type': 'stdout'})
            return

        src_file_path = os.path.join(dataset_path, hdf5_file)
        dest_file_path = os.path.join(aug_dataset_path, hdf5_file)

        try:
            shutil.copy(src_file_path, dest_file_path)

            with h5py.File(dest_file_path, 'a') as f:
                # Determine rectangle parameters for this episode (HDF5 file)
                rect_params = []
                image_keys = list(f['observations/images'].keys())
                if image_keys:
                    first_image_dataset = f['observations/images'][image_keys[0]]
                    if first_image_dataset.shape[0] > 0:
                        img_height, img_width, _ = first_image_dataset.shape[1:]
                        rect_params = generate_rect_params(rectangles, img_width, img_height)

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
                        augmented_images.append(np.array(img))
                    
                    image_group[key][...] = np.array(augmented_images)

            socketio_instance.emit('log_augment_dataset', {'log': f'Augmented {hdf5_file}', 'type': 'stdout'})

        except Exception as e:
            log_message = f"Error processing file {hdf5_file}: {e}"
            print(log_message)
            socketio_instance.emit('log_augment_dataset', {'log': log_message, 'type': 'stderr'})
            
    socketio_instance.emit('augmentation_complete', {'dataset_id': aug_dataset_id})
    
    task_control['stop'] = True
    return