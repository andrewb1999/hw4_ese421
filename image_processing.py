import glob
import numpy as np
import cv2
import constants as c


def rescale_frame(frame, percent=200):
    width = int(frame.shape[1] * percent / 100)
    height = int(frame.shape[0] * percent / 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)


def get_bounding_box(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    h, s, v = cv2.equalizeHist(h), cv2.equalizeHist(s), cv2.equalizeHist(v)
    thresh_h = cv2.bitwise_not(cv2.inRange(h, 12, 240))
    thresh_s = cv2.inRange(s, 220, 255)
    thresh_v = cv2.inRange(v, 100, 255)
    mask = thresh_h & thresh_v & thresh_s

    kernel_open = np.ones((5, 5))
    kernel_close = np.ones((20, 20))
    mask_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    mask_close = cv2.morphologyEx(mask_open, cv2.MORPH_CLOSE, kernel_close)
    mask_blurred = cv2.GaussianBlur(mask_close, (5, 5), 0)
    contours, h = cv2.findContours(mask_blurred.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    prev = 0
    contour = contours[0]
    for i in range(len(contours)):
        _, _, w, h = cv2.boundingRect(contours[i])
        if w * h > prev:
            prev = w * h
            contour = contours[i]
    x, y, w, h = cv2.boundingRect(contour)
    return hsv, x, y, w, h


def get_world_coordinates(x, y, w, h):
    x_c = (c.FOCAL_LENGTH_MM*c.CONE_HEIGHT_MM*c.IMAGE_HEIGHT_PIXELS)/(h*c.SENSOR_HEIGHT_MM)
    y_mm = ((x - c.IMAGE_WIDTH_PIXELS/2)*c.SENSOR_WIDTH_MM)/c.IMAGE_WIDTH_PIXELS
    y_c = (y_mm*x_c)/c.FOCAL_LENGTH_MM
    return x_c, y_c


def main():
    # link to article I used: https://medium.com/@muskulpesent/create-numpy-array-of-images-fecb4e514c4b
    files = glob.glob("pics/*.jpg")  # modify this image path according to what you have
    for my_file in files:
        img = cv2.imread(my_file)
        print(my_file)
        try:
            hsv, x, y, w, h = get_bounding_box(img)
        except IndexError:
            continue
        x_c, y_c = get_world_coordinates(x, y, w, h)
        print(x_c, y_c)
        out = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        cv2.rectangle(out, (x, y), (x + w, y + h), (255, 0, 0), 2)
        img = rescale_frame(out)
        cv2.imshow('image', img)
        cv2.waitKey()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
