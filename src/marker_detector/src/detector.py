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
        for array in corners:
            rect = cv2.boundingRect(array)
            rects.append(rect)

        # aruco.drawDetectedMarkers(img_out, rejectedImgPoints,
        #                          borderColor=(100, 0, 240))
        return (ids, rects)


# image_name = "test"
# image_ext = ".jpg"
# 
# AMOUNT_OF_IMG = 4
# SIZE = 0.2
# DICT = 16
# cv2.namedWindow('image')
# detector = Detector(16)
# for number in range(1, 5):
#     path = "../testfiles/{:s}{:d}{:s}".format(image_name, number, image_ext)
#     img_in = cv2.imread(path)
#     width = img_in.shape[0]
#     height = img_in.shape[1]
#     dim = (int(width*SIZE), int(height*SIZE))
#     img_in = cv2.resize(img_in, dim)
#     img_out = img_in
# 
#     ids, rects = detector.findMarkers(img_in)
# 
#     for rect in rects:
#         cv2.rectangle(img_out, rect, (0, 255, 0), 1)
# 
#     cv2.imshow('image', img_out)
#     cv2.waitKey(0)
# cv2.destroyAllWindows()

