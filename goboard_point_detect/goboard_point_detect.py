from cv2 import cv2
import numpy as np

# 이미지 전처리(Closing 작업)
img = cv2.imread("goboard2.jpg")
img = cv2.resize(img, (640, 640), interpolation=cv2.INTER_CUBIC)
img = cv2.GaussianBlur(img, (5, 5), 0)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
mask = np.zeros(gray.shape, np.uint8)
kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))

close = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel1)
div = np.float32(gray)/(close)
res = np.uint8(cv2.normalize(div, div, 0, 255, cv2.NORM_MINMAX))
res2 = cv2.cvtColor(res, cv2.COLOR_GRAY2BGR)

# 오목판 사각형 찾기 및 마스크 이미지 만들기
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

# 수직선 찾기(Vertical)
kernelx = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 10))

dx = cv2.Sobel(res, cv2.CV_16S, 1, 0)
dx = cv2.convertScaleAbs(dx)
cv2.normalize(dx, dx, 0, 255, cv2.NORM_MINMAX)
ret, close = cv2.threshold(dx, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
close = cv2.morphologyEx(close, cv2.MORPH_DILATE, kernelx, iterations = 1)

contour, hier = cv2.findContours(close, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
for cnt in contour:
    x, y, w, h = cv2.boundingRect(cnt)
    if h/w > 9:
        cv2.drawContours(close, [cnt], 0, 255, -1)
    else:
        cv2.drawContours(close, [cnt], 0, 0, -1)
close = cv2.morphologyEx(close, cv2.MORPH_CLOSE, None, iterations = 2)
closex = close.copy()

# 수평선 찾기(Horizon)
kernely = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 2))

dy = cv2.Sobel(res, cv2.CV_16S, 0, 2)
dy = cv2.convertScaleAbs(dy)
cv2.normalize(dy, dy, 0, 255, cv2.NORM_MINMAX)
ret, close = cv2.threshold(dy, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
close = cv2.morphologyEx(close, cv2.MORPH_DILATE, kernely)

contour, hier = cv2.findContours(close, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
for cnt in contour:
    x, y, w, h = cv2.boundingRect(cnt)
    if w/h > 9:
        cv2.drawContours(close, [cnt], 0, 255, -1)
    else:
        cv2.drawContours(close, [cnt], 0, 0, -1)
close = cv2.morphologyEx(close, cv2.MORPH_DILATE, None, iterations = 2)
closey = close.copy()

# 수직, 수평선을 and하여 오목판의 그리드 서칭
res = cv2.bitwise_and(closex, closey)

# 서칭된 그리드에서의 중심을 찾기위해 모멘트 연산 진행
contour, hier = cv2.findContours(res, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
centroids = []
for cnt in contour:
    mom = cv2.moments(cnt)
    (x, y) = int(mom["m10"]/mom["m00"]), int(mom["m01"]/mom["m00"])
    cv2.circle(img, (x, y), 4, (0, 255, 0), -1)
    centroids.append((x, y))

# 중심 정렬
centroids = np.array(centroids, dtype=np.float32)
c = centroids.reshape((361, 2))
c2 = c[np.argsort(c[:, 1])]

b = np.vstack([c2[i*19:(i+1)*19][np.argsort(c2[i*19:(i+1)*19, 0])] for i in range(19)])
bm = b.reshape((19, 19, 2))

labeled_in_order = res2.copy()

for index, pt in enumerate(b):
    cv2.putText(labeled_in_order, str(index), (int(pt[0]), int(pt[1])),cv2.FONT_HERSHEY_DUPLEX, 0.32, (0, 255, 0))
    cv2.circle(labeled_in_order, (int(pt[0]), int(pt[1])), 3, (0, 0, 255))

# 관점을 바꾸기 위한 PerspectiveTransform 진행
output = np.zeros((640, 640, 3), np.uint8)
srcPoint = np.array([[b[0][0], b[0][1]], [b[18][0], b[18][1]], [b[360][0], b[360][1]], [b[342][0], b[342][1]]], dtype=np.float32)
dstPoint = np.array([[0, 0], [640, 0], [640, 640], [0, 640]], dtype=np.float32)

matrix = cv2.getPerspectiveTransform(srcPoint, dstPoint)
res2 = cv2.warpPerspective(res2, matrix, (640, 640))

cv2.imshow("res2", res2)
cv2.imshow("res", res)
cv2.imshow("closex", closex)
cv2.imshow("closey", closey)
cv2.imshow("img", img)
cv2.imshow("labeled_in_order", labeled_in_order)
cv2.waitKey(0)
cv2.destroyAllWindows()