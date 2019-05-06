import cv2


class Detector:
    def __init__(self):
        self.factor = 1.0

    def find_cold(self, img_in, bounding_box):
        x, y, width, height = bounding_box
        x *= round(self.factor)
        y *= round(self.factor)
        width *= round(1/self.factor)
        height *= round(1/self.factor)

    def is_cold(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow('aaa', image)
        cv2.waitKey()
        mask = cv2.inRange(image, 0, 100)
        print(cv2.countNonZero(mask))
        cv2.imshow('bbb', image)
        cv2.waitKey()

        return True


if __name__ == "__main__":
    # 139 123 250 28 175
    print("A")
    img_in = cv2.imread("brr.png")
    img_gin = cv2.imread('nietbrr.png')
    d = Detector()
    d.is_cold(img_in)
    d.is_cold(img_gin)
