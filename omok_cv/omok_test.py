import numpy as np
from tensorflow.keras.models import load_model
import cv2
from goboard_edge_detect import goboard_edge_detect_module, centroids_sort, index_to_coordinate, data_stone_package
from gostone_matching import gostone_matching_module, stone_55_list

def UI_board_make():
    UI_board = np.zeros((700, 600, 3), np.uint8)
    UI_board[30:570, 30:570] = (137, 153, 168)
    for i in range(1, 20):
        UI_board = cv2.line(UI_board, (30 * i, 30), (30 * i, 570), (0, 0, 0), 2)
        UI_board = cv2.line(UI_board, (30, 30 * i), (570, 30 * i), (0, 0, 0), 2)
    return UI_board

def game_rule(board, player):
    game_result = 0
    diag_line = np.zeros(5)

    for i_idx in range(len(board)):
        for j_idx in range(len(board) - 4):
            p1 = (board[i_idx, j_idx:j_idx + 5] == player)

            if p1.sum() == 5:
                game_result = 1
                return game_result

    for i_idx in range(len(board) - 4):
        for j_idx in range(len(board)):
            p1 = (board[i_idx:i_idx + 5, j_idx] == player)

            if p1.sum() == 5:
                game_result = 1
                return game_result

    for i_idx in range(len(board) - 4):
        for j_idx in range(len(board) - 4):
            diag_line[0] = board[i_idx + 0, j_idx + 0]
            diag_line[1] = board[i_idx + 1, j_idx + 1]
            diag_line[2] = board[i_idx + 2, j_idx + 2]
            diag_line[3] = board[i_idx + 3, j_idx + 3]
            diag_line[4] = board[i_idx + 4, j_idx + 4]

            p1 = (diag_line == player)

            if p1.sum() == 5:
                game_result = 1
                return game_result

    for i_idx in range(len(board) - 4):
        for j_idx in range(len(board) - 4):
            diag_line[0] = board[i_idx + 4, j_idx + 0]
            diag_line[1] = board[i_idx + 3, j_idx + 1]
            diag_line[2] = board[i_idx + 2, j_idx + 2]
            diag_line[3] = board[i_idx + 1, j_idx + 3]
            diag_line[4] = board[i_idx + 0, j_idx + 4]

            p1 = (diag_line == player)

            if p1.sum() == 5:
                game_result = 1
                return game_result
    return game_result

if __name__ == '__main__':
    w, h = 20, 20

    model = load_model('20210307_232530.h5')

    size_of_board = 20
    board_array = np.zeros((size_of_board, size_of_board), dtype=np.int)
    # board_array = [[0 for x in range(20)] for y in range(20)]
    print(board_array)
    black_player = 1
    white_player = 2
    game_result = 0

    max_turn = size_of_board * size_of_board

    black_stone_n = 0
    white_stone_n = 0

    centroids = []
    check_stone = []
    board_buttons = [[0 for x in range(19)] for y in range(19)]
    board_buttons_compare = [[0 for x in range(19)] for y in range(19)]

    all_white_stone = []
    all_black_stone = []

    cap = cv2.VideoCapture(0)
    flag = True

    UI_board = UI_board_make()

    while True:
        key = cv2.waitKey(10)

        check, frame = cap.read()
        img_realtime = frame.copy()
        img_realtime = cv2.resize(img_realtime, (640, 640), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("img_realtime", img_realtime)

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

                for i in range(19):
                    for j in range(19):
                        if board_buttons[i][j] != board_buttons_compare[i][j]:
                            now_black_stone = ([i, j])
                            all_black_stone.append(now_black_stone)
                            print("all_black_stone =", all_black_stone)
                            pos_H, pos_W = i, j

                            board_array[pos_H, pos_W] = 1
                            black_stone_n = black_stone_n + 1
                            game_result = game_rule(board_array, black_player)

                            data_output_black = data_stone_package(i, j)
                            print("현재 착수된 검은돌의 위치", now_black_stone, data_output_black)
                            for i, j in all_black_stone:
                                UI_board = cv2.circle(UI_board, (30*j, 30*i), 13, (0, 0, 0), -1)
                            # cv2.putText(UI_board, "Black ="+str(now_black_stone), (50, 600), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                            cv2.imshow("UI", UI_board)
                            board_buttons_compare[i][j] = board_buttons[i][j]

                if black_stone_n > white_stone_n:
                    if board_buttons == board_buttons_compare:
                        input_1 = board_array.copy()
                        input_1[(input_1 != 1) & (input_1 != 0)] = -1
                        input_1[(input_1 == 1) & (input_1 != 0)] = 1
                        input_1 = np.expand_dims(input_1, axis=(0, -1)).astype(np.float32)

                        output = model.predict(input_1).squeeze()
                        output = output.reshape((h, w))
                        output_y, output_x = np.unravel_index(np.argmax(output), output.shape)
                        data_output_white = data_stone_package(output_y, output_x)
                        now_white_stone = ([output_y, output_x])
                        all_white_stone.append(now_white_stone)
                        print("all_white_stone =", all_white_stone)
                        print("현재 착수된 백돌의 위치", now_white_stone, data_output_white)

                        for i, j in all_white_stone:
                            UI_board = cv2.circle(UI_board, (30 * j, 30 * i), 13, (255, 255, 255), -1)
                        # cv2.putText(UI_board, "White =" + str(now_white_stone), (50, 650), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.imshow("UI", UI_board)

                        pos_H, pos_W = int(output_x), int(output_y)
                        board_array[pos_W, pos_H] = 2  # black is one
                        white_stone_n = white_stone_n + 1
                        game_result = game_rule(board_array, white_player)

            print(game_result)

            print(board_array)

            cv2.imshow("img_board", img_board)
            cv2.imshow("img_rgb", img_rgb)

        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


