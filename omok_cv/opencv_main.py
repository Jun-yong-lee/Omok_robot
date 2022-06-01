import cv2
import numpy as np

from goboard_edge_detect import goboard_edge_detect_module, centroids_sort, index_to_coordinate, data_stone_package
from gostone_matching import gostone_matching_module, stone_55_list

if __name__ == '__main__':

    centroids = []
    check_stone = []
    board_buttons = [[0 for x in range(19)] for y in range(19)]
    board_buttons_compare = [[0 for x in range(19)] for y in range(19)]

    cap = cv2.VideoCapture(0)
    flag = True

    while True:
        key = cv2.waitKey(10)
        check, frame = cap.read()
        img_realtime = frame.copy()
        img_realtime = cv2.resize(img_realtime, (640, 640), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("img_realtime", img_realtime)

        if key == ord('a'):
            print(board_buttons)

        if check and key == ord('q'):
            img = frame.copy()
            dst, img_rgb, stone_list = gostone_matching_module(img)
            stone_coordinate_list = stone_55_list(stone_list)

            if len(centroids) != 361:
                img_board, centroids = goboard_edge_detect_module(img)
            else:
                if flag == True:
                    b = centroids_sort(centroids)
                    flag = False

                for index, pt in enumerate(b):
                    cv2.putText(img_board, str(index), (int(pt[0]), int(pt[1])), cv2.FONT_HERSHEY_DUPLEX, 0.3, (0, 255, 0))
                    cv2.circle(img_board, (int(pt[0]), int(pt[1])), 3, (0, 0, 255))
                    check1 = list([int(pt[0]), int(pt[1])])

                    if len(stone_coordinate_list) != len(check_stone):
                        for stone in stone_coordinate_list:
                            if check1 in stone:
                                x, y = index_to_coordinate(index)
                                board_buttons[x][y] = 1
                                check_stone.append((x, y))
                                check_stone = list(set(check_stone))
                print("착수된 검은돌의 모든 위치 =", check_stone)

                if board_buttons == board_buttons_compare:
                    print("q를 눌러 카메라로 검은돌을 인식해주세요!!")

                for i in range(19):
                    for j in range(19):
                        if board_buttons[i][j] != board_buttons_compare[i][j]:
                            now_stone = ([i, j])
                            data_output = data_stone_package(i, j)
                            print("현재 착수된 검은돌의 위치", now_stone, data_output)
                            board_buttons_compare[i][j] = board_buttons[i][j]


            cv2.imshow("img_board", img_board)
            cv2.imshow("img_rgb", img_rgb)

        key = cv2.waitKey(1)
        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

