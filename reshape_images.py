import cv2
import os
from tqdm import tqdm


def reshape_images(input_dir, output_dir, image_size=128):
    if os.path.exists(input_dir) is False:
        print("input directory is null")
        exit(0)

    classes_dir = [os.path.join(input_dir, el) for el in os.listdir(input_dir)]

    for class_dir in classes_dir:

        objects_dir = [os.path.join(class_dir, el) for el in os.listdir(class_dir) if
                       os.path.isdir(os.path.join(class_dir, el))]

        for objct_dir in tqdm(objects_dir):

            image_dir = os.path.join(objct_dir, 'rendering')
            img_dir = os.path.join(os.path.join(output_dir, os.path.split(class_dir)[-1]), os.path.split(objct_dir)[-1])

            if not os.path.exists(os.path.join(output_dir, os.path.split(class_dir)[-1])):
                os.makedirs(os.path.join(output_dir, os.path.split(class_dir)[-1]))

            image_paths = [os.path.join(image_dir, el) for el in os.listdir(image_dir) if el.endswith('.png')]

            for image_path in image_paths:
                image_name = os.path.basename(image_path)
                origin_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
                resize_image = cv2.resize(origin_image, (image_size, image_size))
                output_path = img_dir + '_' + image_name
                cv2.imwrite(output_path, resize_image)


if __name__ == '__main__':
    input_dir = './ShapeNetRendering'
    output_dir = './ShapeNetImages'
    reshape_images(input_dir, output_dir)
