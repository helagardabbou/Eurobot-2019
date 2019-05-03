import cv2
import numpy as np
from PIL import Image
from sklearn.cluster import KMeans
import colorsys
import time


def Get_Color(data):
    # Convert RGB to HSV.
    # Red, Green, Blue => Hue|0-360|(Angle), Saturation|0-100|, Value|0-100|.
    h, s, v = colorsys.rgb_to_hsv(data[0] / 255, data[1] / 255, data[2] / 255)
    h, s, v = 360 * h, 100 * s, 100 * v

    # List of the angles that match each color.
    liste_h = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255, 270, 285, 300, 315,
               330, 345]
    liste_colors = ["Mid Red", "Warm Red", "Orange", "Warm Yellow", "Mid Yellow", "Cool Yellow", "Yellow Green",
                      "Warm Green", "Mid Green", "Cool Green", "Green Cyan", "Warm Cyan", "Mid Cyan", "Cool Cyan",
                      "Blue Cyan", "Cool Blue", "Mid Blue", "Warm Blue", "Violet", "Cool Magenta", "Mid Magenta",
                      "Warm Magenta", "Red Magenta", "Cool Red"]

    # Match the angle with the nearest basic angle.
    color = []
    if (h >= 360 - 7.5) & (h <= 360):
        color += [liste_colors[0]]
    elif (h >= 0 - 7.5) & (h <= 0 + 7.5):
        color += [liste_colors[0]]
    c = 1
    while c < len(liste_colors):
        if (h >= liste_h[c] - 7.5) & (h <= liste_h[c] + 7.5):
            color += [liste_colors[c]]
        c += 1

    # List of the h and v that we have make vary from 0 to 100 with an increment of 10.
    liste_h_v = [(0, 0), (10, 0), (20, 0), (30, 0), (40, 0), (50, 0), (60, 0), (70, 0), (80, 0), (90, 0), (100, 0),
                 (0, 10), (10, 10), (20, 10), (30, 10), (40, 10), (50, 10), (60, 10), (70, 10), (80, 10), (90, 10),
                 (100, 10), (0, 20), (10, 20), (20, 20), (30, 20), (40, 20), (50, 20), (60, 20), (70, 20), (80, 20),
                 (90, 20), (100, 20), (0, 30), (10, 30), (20, 30), (30, 30), (40, 30), (50, 30), (60, 30), (70, 30),
                 (80, 30), (90, 30), (100, 30), (0, 40), (10, 40), (20, 40), (30, 40), (40, 40), (50, 40), (60, 40),
                 (70, 40), (80, 40), (90, 40), (100, 40), (0, 50), (10, 50), (20, 50), (30, 50), (40, 50), (50, 50),
                 (60, 50), (70, 50), (80, 50), (90, 50), (100, 50), (0, 60), (10, 60), (20, 60), (30, 60), (40, 60),
                 (50, 60), (60, 60), (70, 60), (80, 60), (90, 60), (100, 60), (0, 70), (10, 70), (20, 70), (30, 70),
                 (40, 70), (50, 70), (60, 70), (70, 70), (80, 70), (90, 70), (100, 70), (0, 80), (10, 80), (20, 80),
                 (30, 80), (40, 80), (50, 80), (60, 80), (70, 80), (80, 80), (90, 80), (100, 80), (0, 90), (10, 90),
                 (20, 90), (30, 90), (40, 90), (50, 90), (60, 90), (70, 90), (80, 90), (90, 90), (100, 90),
                 (0, 100),
                 (10, 100), (20, 100), (30, 100), (40, 100), (50, 100), (60, 100), (70, 100), (80, 100), (90, 100),
                 (100, 100)]

    # List that give True if at this point the pixel is black and False if not.
    is_black = [True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True,
                True,
                True, True, True, True, True, False, False, False, False, False, False, False, False, True, True,
                True,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False]

    # List that give True if at this point the pixel is white and False if not.
    is_white = [False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, False, False, False, False, False,
                False,
                False, False, False, False, False, False, False, False, False, True, False, False, False, False,
                False,
                False, False, False, False, False, True, True, True, False, False, False, False, False, False,
                False,
                False]

    # Find in the "liste_h_v" the best match with the (s, v) tuple.
    # Then, if the tuple that we obtain is True in the is_black list, we return "Black".
    # Else if the tuple that we obtain is True in the is_white list, we return "White".
    # Else, we return the color that we have determined with the Hue(angle).
    if is_black[liste_h_v.index((int(10 * round(s / 10, 0)), int(10 * round(v / 10, 0))))] is True:
        return ["Black"]
    elif is_white[liste_h_v.index((int(10 * round(s / 10, 0)), int(10 * round(v / 10, 0))))] is True:
        return ["White"]
    else:
        return color


cap = cv2.VideoCapture(0)

while(True):
    avant = time.time()
    ret, frame = cap.read()

    cv2.imshow('frame', frame)
    data = list(cv2.resize(frame, dsize=(50, 50), interpolation=cv2.INTER_CUBIC))
    data = np.asarray(data)
    data = data[..., ::-1]

    size = (50, 50)
    # Load the image.

    imd = Image.fromarray(data).resize(size, Image.ANTIALIAS)
    # Use KMeans classifier to determine the rgb pixel that sums up the image.
    clf = KMeans(n_clusters=1)
    clf.fit(np.asarray(list(imd.getdata())))
    resultat = int(clf.cluster_centers_[0][0]), int(clf.cluster_centers_[0][1]), int(clf.cluster_centers_[0][2])
    print(Get_Color(resultat))
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break
    #print(time.time()-avant)
cap.release()
cv2.destroyAllWindows()
