import cv2
import numpy as np

def gostone_matching_module(img):
    global dst, img_rgb
    img_rgb = cv2.resize(img, (640, 480), interpolation=cv2.INTER_CUBIC)
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    # 조명의 밝기에 따른 템플릿 이미지를 변경하며 사용할 것
    template = cv2.imread("stone_temp_.PNG", 0)
    template = cv2.resize(template, (20, 20), interpolation=cv2.INTER_CUBIC)
    mask = np.zeros(img_gray.shape, np.uint8)
    mask1 = np.zeros(img_rgb.shape, np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    close = cv2.morphologyEx(img_gray, cv2.MORPH_CLOSE, kernel)
    div = np.float32(img_gray) / close
    res_box = np.uint8(cv2.normalize(div, div, 0, 255, cv2.NORM_MINMAX))
    res_box2 = cv2.cvtColor(res_box, cv2.COLOR_GRAY2BGR)

    thresh = cv2.adaptiveThreshold(res_box, 255, 0, 1, 19, 2)
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

    res_box = cv2.bitwise_and(res_box, mask)
    res_real = cv2.bitwise_and(img_gray, mask)

    # cv2.imshow("res_real", res_real)

    w, h = template.shape[::-1]

    res = cv2.matchTemplate(res_real, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.58
    loc = np.where(res > threshold)
    stone_list = []
    try:
        for pt in zip(*loc[::-1]):
            # cv2.rectangle(img_rgb, pt, (pt[0]+w, pt[1]+h), (0, 255, 0), 1)
            cv2.circle(img_rgb, (pt[0] + w // 2, pt[1] + h // 2), 10, (0, 255, 0), 1)
            mask1[pt[1], pt[0]] = [255, 255, 255]
        mask1 = cv2.cvtColor(mask1, cv2.COLOR_BGR2GRAY)
        cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(mask1)

        dst = cv2.cvtColor(mask1, cv2.COLOR_GRAY2BGR)

        for i in range(1, cnt):
            (x, y, w, h, area) = stats[i]
            cv2.circle(dst, (x, y), 2, (0, 255, 255), 2)
            stone_list.append((x+10, y+10)) # 10만큼의 차이 없애는 작업
    except:
        pass

    return dst, img_rgb, stone_list

def stone_55_list(stone_list):
    temp = []
    stone_coordinate_list = []
    for stone in stone_list:
        # [[temp.append([stone[0] + i - 4, stone[1] + j - 4]) for j in range(0, 9)] for i in range(0, 9)]
        [[temp.append([stone[0] + i - 3, stone[1] + j - 3]) for j in range(0, 7)] for i in range(0, 7)]

        stone_coordinate_list.append(temp)

    return stone_coordinate_list

######################################################################################################
if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    # stone_coordinate_list = []
    temp = []

    while True:
        key = cv2.waitKey(10)
        check, frame = cap.read()
        cv2.imshow("frame", frame)

        if check and key == ord('q'):
            img = frame.copy()
            dst, img_rgb, stone_list = gostone_matching_module(img)

            stone_coordinate_list = stone_55_list(stone_list)
            print(f"stone_list = {stone_list}")
            print(stone_coordinate_list)

            cv2.imshow("dst", dst)
            cv2.imshow("img_rgb", img_rgb)

            key = cv2.waitKey(10)
            if key == 27:
                break
    cap.release()
    cv2.destroyAllWindows()
