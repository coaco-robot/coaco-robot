import cv2
import cv2.aruco as aruco


def dict_track(x):
    global DICT
    DICT = x


def findMarkers(img_in, img_out, DICT):
    aruco_dict = aruco.Dictionary_get(DICT)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(
                img_in, aruco_dict, parameters=parameters)
    # print(corners, ids, rejectedImgPoints)
    print("Using dict: {:d}".format(DICT))
    print(ids)
    aruco.drawDetectedMarkers(img_out, corners, ids)
    aruco.drawDetectedMarkers(img_out, rejectedImgPoints,
                              borderColor=(100, 0, 240))


image_name = "test"
image_ext = ".jpg"

AMOUNT_OF_IMG = 4
SIZE = 0.2
DICT = 4
cv2.namedWindow('image')
cv2.createTrackbar('DICT', 'image', 0, 20, dict_track)
while True:
    for number in range(1, 5):
        path = "{:s}{:d}{:s}".format(image_name, number, image_ext)
        img_in = cv2.imread(path)
        width = img_in.shape[0]
        height = img_in.shape[1]
        dim = (int(width*SIZE), int(height*SIZE))
        img_in = cv2.resize(img_in, dim)
        img_out = img_in

        findMarkers(img_in, img_out, DICT)

        cv2.waitKey(0)
        cv2.imshow('image', img_out)
cv2.destroyAllWindows()
