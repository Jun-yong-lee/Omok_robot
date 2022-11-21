import numpy as np
import time
import tensorflow as tf
from tensorflow.keras.models import load_model


def solved_cublas_status_alloc_failed():

    gpu = tf.config.experimental.list_physical_devices('GPU')  # 내 컴에 장착된 GPU를 list로 반환
    try:
        tf.config.experimental.set_memory_growth(gpu[0], True)  # GPU Memory Growth를 Enable
    except RuntimeError as e:
        print(e)  # Error 발생하면 Error 내용 출력


def game_rule(board, player):
    """
    :param board: omok board (0 -> empty | 1 -> black | 2 -> white)
    :param player: players to check
    :return: result of the game
    """
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


def ai_turn(turn_select, board_array):
    """
    :param turn_select: 1st -> black(1), 2st -> white(2)
    :param board_array: board array in omok game
    :return: laid out board array
    """
    print("AI turn")
    time.sleep(2)
    input_1 = board_array.copy()
    input_1[(input_1 != 1) & (input_1 != 0)] = -1
    input_1[(input_1 == 1) & (input_1 != 0)] = 1
    input_1 = np.expand_dims(input_1, axis=(0, -1)).astype(np.float32)
    output = model.predict(input_1).squeeze()
    output = output.reshape((h, w))
    output_y, output_x = np.unravel_index(np.argmax(output), output.shape)
    board_array[output_y, output_x] = turn_select
    game_rule(board_array, turn_select)
    print("AI", output_y, output_x)
    print("AI가 예측한 수의 정확도 :", output[output_y, output_x])
    print(board_array)
    print('\n')
    return board_array


def player_turn(turn_select, board_array):
    """
    :param turn_select: 1st -> black(1), 2st -> white(2)
    :param board_array: board array in omok game
    :return: laid out board array
    """
    print("My turn, Example of input y,x ==> 1,1")
    position = input()
    sub_p = position.split(',')
    pos_h, pos_w = int(sub_p[0]), int(sub_p[1])
    board_array[pos_h, pos_w] = turn_select
    game_rule(board_array, turn_select)
    print(board_array)
    print('\n')
    return board_array


if __name__ == '__main__':
    solved_cublas_status_alloc_failed()
    w, h = 19, 19

    # model = load_model('cnn_omok_AI.h5')
    model = load_model('20221101_213945_1919_50.h5')

    size_of_board = 19
    board_array = np.zeros((size_of_board, size_of_board), dtype=np.int)

    game_result = 0

    turn_select = int(input("선공 정하기, AI-->1, USER-->2 "))
    print("Selected turn :", turn_select)

    max_turn = size_of_board * size_of_board

    if turn_select == 1:
        for i in range(max_turn):
            if i % 2 == 0:
                board_array = ai_turn(1, board_array)
            else:
                board_array = player_turn(2, board_array)

    else:
        for i in range(max_turn):
            if i % 2 == 0:
                board_array = player_turn(1, board_array)
            else:
                board_array = ai_turn(2, board_array)

