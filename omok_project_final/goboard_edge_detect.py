import cv2
import numpy as np

def goboard_edge_detect_module(img):

    # global img_board, labeled_in_order, pt

    img_board = img.copy()
    img_board = cv2.resize(img_board, (640, 480), interpolation=cv2.INTER_CUBIC)
    img_board = cv2.GaussianBlur(img_board, (5, 5), 0)
    gray = cv2.cvtColor(img_board, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(gray.shape, np.uint8)
    kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))

    close = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel1)
    div = np.float32(gray) / (close)
    res = np.uint8(cv2.normalize(div, div, 0, 255, cv2.NORM_MINMAX))
    res2 = cv2.cvtColor(res, cv2.COLOR_GRAY2BGR)

    thresh = cv2.adaptiveThreshold(res, 255, 0, 1, 19, 2)
    contour, hier = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    best_cnt = None
    for cnt in contour:
        area = cv2.contourArea(cnt)
        if area > 1000:
            if area > max_area:
                max_area = area
                best_cnt = cnt
    cv2.drawContours(mask, [best_cnt], 0, 255, -1)
    cv2.drawContours(mask, [best_cnt], 0, 0, 2)

    res = cv2.bitwise_and(res, mask)

    kernelx = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 10))

    dx = cv2.Sobel(res, cv2.CV_16S, 1, 0)
    dx = cv2.convertScaleAbs(dx)
    cv2.normalize(dx, dx, 0, 255, cv2.NORM_MINMAX)
    ret, close = cv2.threshold(dx, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    close = cv2.morphologyEx(close, cv2.MORPH_DILATE, kernelx, iterations=1)

    contour, hier = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contour:
        x, y, w, h = cv2.boundingRect(cnt)
        if h / w > 9:
            cv2.drawContours(close, [cnt], 0, 255, -1)
        else:
            cv2.drawContours(close, [cnt], 0, 0, -1)
    close = cv2.morphologyEx(close, cv2.MORPH_CLOSE, None, iterations=2)
    closex = close.copy()

    kernely = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 2))

    dy = cv2.Sobel(res, cv2.CV_16S, 0, 2)
    dy = cv2.convertScaleAbs(dy)
    cv2.normalize(dy, dy, 0, 255, cv2.NORM_MINMAX)
    ret, close = cv2.threshold(dy, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    close = cv2.morphologyEx(close, cv2.MORPH_DILATE, kernely)

    contour, hier = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contour:
        x, y, w, h = cv2.boundingRect(cnt)
        if w / h > 9:
            cv2.drawContours(close, [cnt], 0, 255, -1)
        else:
            cv2.drawContours(close, [cnt], 0, 0, -1)
    close = cv2.morphologyEx(close, cv2.MORPH_DILATE, None, iterations=2)
    closey = close.copy()

    res = cv2.bitwise_and(closex, closey)

    contour, hier = cv2.findContours(res, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    centroids = []
    for cnt in contour:
        mom = cv2.moments(cnt)
        try:
            (x, y) = int(mom["m10"] / mom["m00"]), int(mom["m01"] / mom["m00"])
        except:
            pass
        cv2.circle(img_board, (x, y), 4, (0, 255, 0), -1)
        centroids.append((x, y))
    if len(centroids) == 361:
        centroids = np.array(centroids, dtype=np.float32)
        c = centroids.reshape((361, 2))
        c2 = c[np.argsort(c[:, 1])]

        b = np.vstack([c2[i * 19:(i + 1) * 19][np.argsort(c2[i * 19:(i + 1) * 19, 0])] for i in range(19)])
        bm = b.reshape((19, 19, 2))

        labeled_in_order = res2.copy()

        for index, pt in enumerate(b):
            cv2.putText(labeled_in_order, str(index), (int(pt[0]), int(pt[1])), cv2.FONT_HERSHEY_DUPLEX, 0.32, (0, 255, 0))
            cv2.circle(labeled_in_order, (int(pt[0]), int(pt[1])), 3, (0, 0, 255))

    return img_board, centroids

def centroids_sort(centroids):
    centroids = np.array(centroids, dtype=np.float32)
    c = centroids.reshape((361, 2))
    c2 = c[np.argsort(c[:, 1])]

    b = np.vstack([c2[i * 19:(i + 1) * 19][np.argsort(c2[i * 19:(i + 1) * 19, 0])] for i in range(19)])
    bm = b.reshape((19, 19, 2))
    return b

def index_to_coordinate(index):
    x = index//19
    y = index%19
    return x, y

def data_stone_package(x, y):
    data_x = str(x)
    data_y = str(y)

    if len(data_x) != 2:
        data_x = '0' + data_x
    if len(data_y) != 2:
        data_y = '0' + data_y
    data_output = "Q" + data_x + data_y
    return data_output