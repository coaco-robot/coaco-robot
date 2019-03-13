import cv2
import cv2.aruco as aruco


def findMarkers(img_in, img_out):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(
                img_in, aruco_dict, parameters=parameters)
    # print(corners, ids, rejectedImgPoints)
    print(ids)
    aruco.drawDetectedMarkers(img_out, corners, ids)
    aruco.drawDetectedMarkers(img_out, rejectedImgPoints,
                              borderColor=(100, 0, 240))


image_name = "test"
image_ext = ".jpg"

AMOUNT_OF_IMG = 4
SIZE = 0.2
for number in range(1, 5):
    path = "{:s}{:d}{:s}".format(image_name, number, image_ext)
    img_in = cv2.imread(path)
    width = img_in.shape[0]
    height = img_in.shape[1]
    dim = (int(width*SIZE), int(height*SIZE))
    img_in = cv2.resize(img_in, dim)
    img_out = img_in

    findMarkers(img_in, img_out)

    cv2.imshow('aruco marker', img_out)
    cv2.waitKey(0)
cv2.destroyAllWindows()
