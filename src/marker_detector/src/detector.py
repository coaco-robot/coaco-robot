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
        print(ids)
        print(corners)
        rects = []
        img_out = img_in
        for array in corners:
            x,y,w,h = cv2.boundingRect(array)
	    rect = (x,y,w,h)
            rects.append(rect)
            cv2.rectangle(img_out, (x,y), (x+w, y+h), (0,255,0), 3)

        return ids, rects, img_out


