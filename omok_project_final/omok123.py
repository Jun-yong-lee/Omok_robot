import numpy as np
from tensorflow.keras.models import load_model
import cv2
from goboard_edge_detect import goboard_edge_detect_module, centroids_sort, index_to_coordinate, data_stone_package
from gostone_matching import gostone_matching_module, stone_55_list
import pygame, sys
from pygame.locals import *
from gomoku import Board, Gomoku
from rule import *
import threading
from multiprocessing import Process
import multiprocessing as mp

w, h = 20, 20
board_1 = Board(w=w, h=h)
game = Gomoku(board=board_1)
model = load_model('20210307_232530.h5')

check_num = 0
global now_black_stone
global centroids, flag, img_board, check_stone, black_stone_n, white_stone_n
bg_color = (128, 128, 128)
black = (0, 0, 0)
blue = (0, 50, 255)
white = (255, 255, 255)
red = (255, 0, 0)
green = (0, 200, 0)

window_width = 1280
window_height = 800
board_width = 800
grid_size = 40

fps = 60
fps_clock = pygame.time.Clock()


class Omok(object):
    def __init__(self, surface):
        self.board = [[0 for i in range(board_size)] for j in range(board_size)]
        self.menu = Menu(surface)
        self.rule = Rule(self.board)
        self.surface = surface
        self.pixel_coords = []
        self.set_coords()
        self.set_image_font()
        self.is_show = True

    def init_game(self):
        self.turn = black_stone
        self.draw_board()
        self.menu.show_msg(empty)
        self.init_board()
        self.coords = []
        self.redos = []
        self.id = 1
        self.is_gameover = False
        self.is_forbidden = False

    def set_image_font(self):
        black_img = pygame.image.load('image/black.png')
        white_img = pygame.image.load('image/white.png')
        self.last_w_img = pygame.image.load('image/white_a.png')
        self.last_b_img = pygame.image.load('image/black_a.png')
        self.board_img = pygame.image.load('omokBoard.png')
        self.forbidden_img = pygame.image.load('image/forbidden.png')
        self.font = pygame.font.Font("freesansbold.ttf", 14)
        self.black_img = pygame.transform.scale(black_img, (grid_size, grid_size))
        self.white_img = pygame.transform.scale(white_img, (grid_size, grid_size))

    def init_board(self):
        for y in range(board_size):
            for x in range(board_size):
                self.board[y][x] = 0

    def draw_board(self):
        self.surface.blit(self.board_img, (0, 0))

    def draw_image(self, img_index, x, y):
        img = [self.black_img, self.white_img, self.last_b_img, self.last_w_img, self.forbidden_img]
        self.surface.blit(img[img_index], (x + 20, y + 20))

    def show_number(self, x, y, stone, number):
        colors = [white, black, red, red]
        color = colors[stone]
        self.menu.make_text(self.font, str(number), color, None, y + 40, x + 40, 1)

    def hide_numbers(self):
        for i in range(len(self.coords)):
            x, y = self.coords[i]
            self.draw_image(i % 2, x, y)
        if self.coords:
            x, y = self.coords[-1]
            self.draw_image(i % 2 + 2, x, y)

    def show_numbers(self):
        for i in range(len(self.coords)):
            x, y = self.coords[i]
            self.show_number(x, y, i % 2, i + 1)
        if self.coords:
            x, y = self.coords[-1]
            self.draw_image(i % 2, x, y)
            self.show_number(x, y, i % 2 + 2, i + 1)

    def check_forbidden(self):
        if self.turn == black_stone:
            coords = self.rule.get_forbidden_points(self.turn)
            while coords:
                x, y = coords.pop()
                x, y = x * grid_size + 20, y * grid_size + 20
                self.draw_image(4, x, y)
            self.is_forbidden = True

    def draw_stone(self, coord, stone, increase):
        if self.is_forbidden:
            self.draw_board()
        x, y = self.get_point(coord)
        self.board[y][x] = stone
        self.hide_numbers()
        if self.is_show:
            self.show_numbers()
        self.id += increase
        self.turn = 3 - self.turn
        self.check_forbidden()

    def undo(self):
        if not self.coords:
            return
        self.draw_board()
        coord = self.coords.pop()
        self.redos.append(coord)
        self.draw_stone(coord, empty, -1)

    def undo_all(self):
        if not self.coords:
            return
        self.id = 1
        self.turn = black_stone
        while self.coords:
            coord = self.coords.pop()
            self.redos.append(coord)
        self.init_board()
        self.draw_board()

    def redo(self):
        if not self.redos:
            return
        coord = self.redos.pop()
        self.coords.append(coord)
        self.draw_stone(coord, self.turn, 1)

    def set_coords(self):
        for y in range(board_size):
            for x in range(board_size):
                self.pixel_coords.append((x * grid_size, y * grid_size))

    def get_coord(self, pos):
        for coord in self.pixel_coords:
            x, y = coord
            rect = pygame.Rect(x, y, grid_size, grid_size)
            if rect.collidepoint(pos):
                return coord
        return None

    def get_point(self, coord):
        x, y = coord
        x = (x) // grid_size
        y = (y) // grid_size
        return x, y

    def check_board(self, pos):
        coord = self.get_coord(pos)
        if not coord:
            return False
        x, y = self.get_point(coord)
        if self.board[y][x] != empty:
            print("occupied")
            return True

        if self.turn == black_stone:
            if self.rule.forbidden_point(x, y, self.turn):
                print("forbidden point")
                return True

        self.coords.append(coord)
        self.draw_stone(coord, self.turn, 1)
        if self.check_gameover(coord, 3 - self.turn):
            self.is_gameover = True
        if len(self.redos):
            self.redos = []
        return True

    def check_gameover(self, coord, stone):
        x, y = self.get_point(coord)
        if self.id > board_size * board_size:
            self.show_winner_msg(stone)
            return True
        elif self.rule.is_gameover(x, y, stone):
            self.show_winner_msg(stone)
            return True
        return False

    def show_winner_msg(self, stone):
        for i in range(3):
            self.menu.show_msg(stone)
            pygame.display.update()
            pygame.time.delay(200)
            self.menu.show_msg(empty)
            pygame.display.update()
            pygame.time.delay(200)
        self.menu.show_msg(stone)


class Menu(object):
    def __init__(self, surface):
        self.font = pygame.font.Font('freesansbold.ttf', 20)
        self.surface = surface
        self.draw_menu()

    def draw_menu(self):
        top, left = window_height - 30, window_width - 200
        self.new_rect = self.make_text(self.font, 'New Game', blue, None, top - 30, left)
        self.quit_rect = self.make_text(self.font, 'Quit Game', blue, None, top, left)
        self.show_rect = self.make_text(self.font, 'Hide Number  ', blue, None, top - 60, left)
        self.undo_rect = self.make_text(self.font, 'Undo', blue, None, top - 150, left)
        self.uall_rect = self.make_text(self.font, 'Undo All', blue, None, top - 120, left)
        self.redo_rect = self.make_text(self.font, 'Redo', blue, None, top - 90, left)

    def show_msg(self, msg_id):
        msg = {
            empty: '                                    ',
            black_stone: 'Black win!!!',
            white_stone: 'White win!!!',
            tie: 'Tie',
        }
        center_x = window_width - (window_width - board_width) // 2
        self.make_text(self.font, msg[msg_id], black, bg_color, 30, center_x, 1)

    def make_text(self, font, text, color, bgcolor, top, left, position=0):
        surf = font.render(text, False, color, bgcolor)
        rect = surf.get_rect()
        if position:
            rect.center = (left, top)
        else:
            rect.topleft = (left, top)
        self.surface.blit(surf, rect)
        return rect

    def show_hide(self, omok):
        top, left = window_height - 90, window_width - 200
        if omok.is_show:
            self.make_text(self.font, 'Show Number', blue, bg_color, top, left)
            omok.hide_numbers()
            omok.is_show = False
        else:
            self.make_text(self.font, 'Hide Number  ', blue, bg_color, top, left)
            omok.show_numbers()
            omok.is_show = True

    def check_rect(self, pos, omok):
        if self.new_rect.collidepoint(pos):
            return True
        elif self.show_rect.collidepoint(pos):
            self.show_hide(omok)
        elif self.undo_rect.collidepoint(pos):
            omok.undo()
        elif self.uall_rect.collidepoint(pos):
            omok.undo_all()
        elif self.redo_rect.collidepoint(pos):
            omok.redo()
        elif self.quit_rect.collidepoint(pos):
            self.terminate()
        return False

    def terminate(self):
        pygame.quit()
        sys.exit()

    def is_continue(self, omok):
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.terminate()
                elif event.type == MOUSEBUTTONUP:
                    if (self.check_rect(event.pos, omok)):
                        return
            pygame.display.update()
            fps_clock.tick(fps)


def opencv_stone():

    w, h = 20, 20

    model = load_model('20210307_232530.h5')
    global centroids, flag, img_board, check_stone, black_stone_n, white_stone_n
    size_of_board = 20
    board_array = np.zeros((size_of_board, size_of_board), dtype=np.int8)
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

    cap = cv2.VideoCapture(1)  # , cv2.CAP_DSHOW
    flag = True
    check_num = 0
    print('check_num :', check_num)
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
                    cv2.putText(img_board, str(index), (int(pt[0]), int(pt[1])), cv2.FONT_HERSHEY_DUPLEX, 0.3,
                                (0, 255, 0))
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
                            # game_result = game_rule(board_array, black_player)

                            data_output_black = data_stone_package(i, j)
                            print("현재 착수된 검은돌의 위치", now_black_stone, data_output_black)

                            check_num = 1
                            print('check_num :', check_num)
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

                        if check_num == 2:
                            check_num = 1
                            print('check_num :', check_num)

                        pos_H, pos_W = int(output_x), int(output_y)
                        board_array[pos_W, pos_H] = 2  # black is one
                        white_stone_n = white_stone_n + 1

            print(board_array)

            cv2.imshow("img_board", img_board)
            cv2.imshow("img_rgb", img_rgb)

        if key == 27:
            break

    cap.release()

    w, h = 20, 20

    model = load_model('20210307_232530.h5')
    # global centroids, flag, img_board, check_stone, black_stone_n, white_stone_n
    size_of_board = 20
    board_array = np.zeros((size_of_board, size_of_board), dtype=np.int8)
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

    cap = cv2.VideoCapture(1)  # , cv2.CAP_DSHOW
    flag = True
    check_num = 0
    print('check_num :', check_num)
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
                    cv2.putText(img_board, str(index), (int(pt[0]), int(pt[1])), cv2.FONT_HERSHEY_DUPLEX, 0.3,
                                (0, 255, 0))
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
                            # game_result = game_rule(board_array, black_player)

                            data_output_black = data_stone_package(i, j)
                            print("현재 착수된 검은돌의 위치", now_black_stone, data_output_black)

                            check_num = 1
                            print('check_num :', check_num)
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

                        if check_num == 2:
                            check_num = 1
                            print('check_num :', check_num)

                        pos_H, pos_W = int(output_x), int(output_y)
                        board_array[pos_W, pos_H] = 2  # black is one
                        white_stone_n = white_stone_n + 1

            print(board_array)

            cv2.imshow("img_board", img_board)
            cv2.imshow("img_rgb", img_rgb)

        if key == 27:
            break
    cv2.waitKey(1)
    # cap.release()
    # cv2.destroyAllWindows()
#

# def opencv_thread():
#     cv_thread = threading.Thread(target=opencv_stone())
#     cv_thread.daemon = True
#     cv_thread.start()


def main():
    pygame.init()
    surface = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption("Omok game")
    surface.fill(bg_color)
    global check_num
    omok = Omok(surface)
    menu = Menu(surface)

    omok.init_game()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    print('hi')
        # if check_num == 1:
        #     print('오목게임 흑돌 check_num :', check_num)
        #     black_stone_pos_x = now_black_stone[0]
        #     black_stone_pos_y = now_black_stone[1]
        #
        #     black_stone_pos = (black_stone_pos_x * grid_size + 40, black_stone_pos_y * grid_size + 40)
        #     print(black_stone_pos)
        #     omok.check_board(black_stone_pos)
        #
        #     if omok.is_gameover:
        #         return
        #
        #     pygame.display.update()
        #     fps_clock.tick(fps)
        #     check_num = 2
        #     print('check_num :', check_num)
        #
        # elif check_num == 2:
        #     print('check_num :', check_num)
        #     input_board = np.array(omok.board.copy())
        #
        #     print(input_board)
        #     input_board[(input_board != 1) & (input_board != 0)] = -1
        #     input_board[(input_board == 1) & (input_board != 0)] = 1
        #     input_board = np.expand_dims(input_board, axis=(0, -1)).astype(np.float32)
        #
        #     output = model.predict(input_board).squeeze()
        #     output = output.reshape((h, w))
        #     output_y, output_x = np.unravel_index(np.argmax(output), output.shape)
        #
        #     pos = [output_x, output_y]
        #     POS = [output_x * grid_size + 40, output_y * grid_size + 40]
        #     print(pos)
        #     print(POS)
        #     omok.check_board(POS)
        #     check_num = 1
        #     print('check_num :', check_num)
        #     print(np.array(omok.board.copy()))
        #     if omok.is_gameover:
        #         return
        menu.is_continue(omok)
        pygame.display.update()
        fps_clock.tick(fps)
#
# def pygame_s():
#     pygame.init()
#     surface = pygame.display.set_mode((window_width, window_height))
#     pygame.display.set_caption("Omok game")
#     surface.fill(bg_color)
#
#     omok = Omok(surface)
#     menu = Menu(surface)
#     while True:
#         run_game(surface, omok, menu)
#         menu.is_continue(omok)

#
# def run_game(surface, omok, menu):
#     global check_num
#     omok.init_game()
#     while True:
#         menu.is_continue(omok)
#         if check_num == 1:
#             print('오목게임 흑돌 check_num :', check_num)
#             black_stone_pos_x = now_black_stone[0]
#             black_stone_pos_y = now_black_stone[1]
#
#             black_stone_pos = (black_stone_pos_x * grid_size + 40, black_stone_pos_y * grid_size + 40)
#             print(black_stone_pos)
#             omok.check_board(black_stone_pos)
#
#             if omok.is_gameover:
#                 return
#
#             pygame.display.update()
#             fps_clock.tick(fps)
#             check_num = 2
#             print('check_num :', check_num)
#
#         elif check_num == 2:
#             print('check_num :', check_num)
#             input_board = np.array(omok.board.copy())
#
#             print(input_board)
#             input_board[(input_board != 1) & (input_board != 0)] = -1
#             input_board[(input_board == 1) & (input_board != 0)] = 1
#             input_board = np.expand_dims(input_board, axis=(0, -1)).astype(np.float32)
#
#             output = model.predict(input_board).squeeze()
#             output = output.reshape((h, w))
#             output_y, output_x = np.unravel_index(np.argmax(output), output.shape)
#
#             pos = [output_x, output_y]
#             POS = [output_x * grid_size + 40, output_y * grid_size + 40]
#             print(pos)
#             print(POS)
#             omok.check_board(POS)
#             check_num = 1
#             print('check_num :', check_num)
#             print(np.array(omok.board.copy()))
#             if omok.is_gameover:
#                 return
#
#             pygame.display.update()
#             fps_clock.tick(fps)

#
# def pygame_thread():
#     thread = threading.Thread(target=pygame_s())
#     thread.daemon = True
#     thread.start()


if __name__ == '__main__':

    main()
    # pygame_s()
    # opencv_stone()




    # UI_board = UI_board_make()

# TODO CHECK NUM 변수 해야됨 멀티프로세스 해보기
