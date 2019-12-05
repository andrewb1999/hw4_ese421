from picamera.array import PiRGBArray
from picamera import PiCamera
from smbus import SMBus
import struct
import traceback
import time
import cv2
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
    _, contours, _= cv2.findContours(mask_blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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


def process_image(img):
    hsv, x, y, w, h = get_bounding_box(img)
    x_c, y_c = get_world_coordinates(x, y, w, h)
    return x_c, y_c


def main():
    com = ArduinoCommunicator(0x8)
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    time.sleep(0.1)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        try:
            x_c, y_c = process_image(image)
            com.write_float_to_register(x_c, com.data_to_arduino_register["cone_x"])
            com.write_float_to_register(y_c, com.data_to_arduino_register["cone_y"])
        except (IndexError, OSError):
            pass
        finally:
            rawCapture.truncate(0)
            time.sleep(0.5)


class ArduinoCommunicator:
    def __init__(self, arduino_address):
        self.address = arduino_address
        self.bus = SMBus(1)
        self.write_attempt = 0
        self.data_from_arduino = dict.fromkeys(["IMU data"])
        self.UPDATE_SEND_REGISTER = 11

        self.data_to_arduino_register = {
            "cone_x": 1,
            "cone_y": 2
        }

        self.SIZE_OF_ARDUINO_SEND_REGISTERS = 8

    def write_float_to_register(self, num, register):
        """
        Writes a float into the Arduino's receive register
        """

        if register < 0 or register > 7:
            raise Exception("Error in write float: register must be in [0,7]")
        data = list(bytearray(str(num), 'utf8'))
        self._write(register, data)

    def _write(self, register, data):
        timeout = 10
        try:
            self.bus.write_i2c_block_data(self.address, register, data)
            self.write_attempt = 0
        except Exception as e:
            self.write_attempt += 1
            if self.write_attempt < timeout:
                print("Failed to write due to Exception " + str(e) + ". Trying again")
                self._write(register, data)
            else:
                print("Timed out writing")
                traceback.print_exc()


if __name__ == "__main__":
    main()


