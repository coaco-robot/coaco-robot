import cv2
import cv2.aruco as aruco


class Detector:
    def __init__(self, dictionary):
        self.dict = dictionary

    def findMarkers(self, img_in):
        aruco_dict = aruco.Dictionary_get(self.dict)
        parameters = aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(
                    img_in, aruco_dict, parameters=parameters)
        # print(corners, ids, rejectedImgPoints)
        print("Using dict: {:d}".format(self.dict))
        print(ids)
        print(corners)
        rects = []
        img_out = img_in
        for array in corners:
            rect = cv2.boundingRect(array)
            rects.append(rect)
            cv2.rectangle(img_out, rect, (0, 255, 0), 3)

        return ids, rects, img_out


