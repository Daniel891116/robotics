# image process test for HW4
import cv2
import math
import numpy as np
from skimage import measure as skiMeasure
from skimage import color
import matplotlib.pyplot as plt

# Parameter
filtSize = 5
threshold = 40 # ?
kernelSize = 10
org_x = 661
org_y = 785
cube_threshold = (15000,5000)
pxl2mm = 25 / math.sqrt(7743.5)

def img_preprocess(orginalImg):
    img = cv2.cvtColor(orginalImg, cv2.COLOR_RGB2GRAY) # BGR2GRAY

    # filter
    filt = np.ones((filtSize, filtSize), np.float32) / (filtSize**2)
    img = cv2.filter2D(img.astype('float32'), -1, filt, borderType = cv2.BORDER_CONSTANT)

    # to binary
    img = (img > threshold).astype('uint8')

    # clear border, imclearborder
    # image open and close
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelSize, kernelSize)) # MORPH_ELLIPSE, MORPH_CROSS
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    # regionprops
    contours, _ = cv2.findContours(image=img, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_NONE)
    labels = skiMeasure.label(img, connectivity = 2)
    return labels, contours

def cube_locate(img):
    region_cnt = 0
    cubes = []
    labels, contours = img_preprocess(img)
    # regions = skiMeasure.regionprops(labels)
    # for region in regions:
    #     if region.area < cube_threshold:
    #         continue
    #     region_cnt += 1
    #     y0, x0 = region.centroid
    #     orientation = region.orientation
    #     dstx = (x0 - org_x) * pxl2mm
    #     dsty = (y0 - org_y) * pxl2mm
    #     dst_rot = 180*orientation/math.pi
    #     cubes.append([dstx,dsty,dst_rot])
    for cnt in contours:
        if cv2.contourArea(cnt) > cube_threshold[1] and cv2.contourArea(cnt) < cube_threshold[0]:   
            obj = dict()
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])         
            # get rotated rectangle from outer contour
            rotrect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rotrect)
            box = np.int0(box)
            obj['center'] = ((cx - org_x) * pxl2mm, (cy - org_y) * pxl2mm)
            obj['real_center'] = (cx,cy)
            # get angle from rotated rectangle
            angle = rotrect[-1]
            if angle > 45:
                angle -= 90
            elif angle < -45:
                angle += 90
            obj['orientation'] = angle
            cubes.append(obj)
    visualize(img,labels,cubes)
    if len(cubes) == 0:
        return None
    else:
        # TODO
        # try to find the optimal cube to grab
        dstx = cubes[-1]['center'][0]
        dsty = cubes[-1]['center'][1]
        dst_rot = cubes[-1]['orientation']
        return dstx, dsty, dst_rot
        
def visualize(img,labels,cubes):
    # visualize
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4))
    ax1.imshow(img, plt.cm.gray, interpolation='nearest')
    ax1.axis('off')
    dst=color.label2rgb(labels)  # label colors to different groups
    ax2.imshow(dst,interpolation='nearest')
    ax2.axis('off')
    label_map = np.zeros_like(dst)
    print(label_map.shape)
    for obj in cubes:
        angle = obj['orientation'] + 90
        slope = math.tan(angle * math.pi / 180)
        (cx, cy) = obj['real_center']
        # L_point = (0, -cx * slope + cy)
        # R_point = (label_map.shape[1], (label_map.shape[1] - cx) * slope + cy)
        L_point = (cx - 30,cy - 30 * slope)
        R_point = (cx + 30,cy + 30 * slope)
        ax2.plot((L_point[0], R_point[0]), (L_point[1], R_point[1]), color = 'w', linewidth = 1)
    fig.tight_layout()
    plt.show()

# def visualize(img):
#     # assuming getting image "img"
#     labels = img_preprocess(img)
#     # visualize
#     fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4))
#     ax1.imshow(img, plt.cm.gray, interpolation='nearest')
#     ax1.axis('off')
#     dst=color.label2rgb(labels)  # label colors to different groups
#     ax2.imshow(dst,interpolation='nearest')
#     ax2.axis('off')

#     regions = skiMeasure.regionprops(labels)
#     for region in regions:
#         if region.area < cube_threshold:
#             continue
#         print("Centroid",region.centroid)
#         y0, x0 = region.centroid
#         orientation = region.orientation
#         print("Degree", 180*orientation/math.pi)
#         x1 = x0 + math.cos(orientation) * 0.5 * region.axis_minor_length
#         y1 = y0 - math.sin(orientation) * 0.5 * region.axis_minor_length
#         x2 = x0 - math.sin(orientation) * 0.5 * region.axis_major_length
#         y2 = y0 - math.cos(orientation) * 0.5 * region.axis_major_length

#         ax2.plot((x0, x1), (y0, y1), '-r', linewidth=2.5)
#         ax2.plot((x0, x2), (y0, y2), '-r', linewidth=2.5)
#         ax2.plot(x0, y0, '.g', markersize=15)

    # fig.tight_layout()
    # plt.show()


if __name__ == '__main__':
    filename = "testimgs/output_18.png"
    img = cv2.imread(filename)
    print(cube_locate(img))
    # visualize()