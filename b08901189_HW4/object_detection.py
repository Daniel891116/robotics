import cv2 as cv
import numpy as np
import os 
import glob
import errno
import matplotlib.pyplot as plt

from camera import Camera

def main():

    config = {
        #file directory
        'mtx_file': 'in_mtx.npy',
        'img_dir': 'calibrate_imgs',
        # 'img_dir': 'images',
        'img_type': 'png',
        # 'img_type': 'jpg',
        'out_dir': 'output',
        'sample_number': 1,

        # image parameter
        'bi_thres': 180,
        'object_thres': 1000,
        'number_thres': 5
    }    

    if not os.path.exists(config['out_dir']):
        os.mkdir(config['out_dir'])

    if os.path.exists(config['img_dir']):
        files = sorted(glob.glob(os.path.join(os.getcwd(), config['img_dir'], f"*.{config['img_type']}")))
    else:
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), config['img_dir'])

    camera = Camera(config)

    for file_name in files[0:config['sample_number']]:
        fig = plt.figure(figsize=(10, 8))
        ax_1 = fig.add_subplot(1, 1, 1)
        img = cv.imread(file_name)
        (h, w) = img.shape[:2]
        (cX, cY) = (w // 2, h // 2)
        # rotate our image by 45 degrees around the center of the image
        M = cv.getRotationMatrix2D((cX, cY), 0, 1.0)
        img = cv.warpAffine(img, M, (w, h))
        camera.load_img(img)
        camera.binarization()
        camera.find_objects()
        camera.draw_contours(ax_1)

        for i, obj in enumerate(camera.objects):
            ax_1.scatter(*(obj['center']), color = 'r', s = 2)
            print(f"{i+1}th object = {obj['orientation']:.4f} deg")

        ax_1.imshow(camera.label_map, cmap = 'gray')
        # plt.show()
    
    camera.calibration()


if __name__ == '__main__':
    main()