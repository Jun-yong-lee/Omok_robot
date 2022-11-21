import pygame
import pandas

from omok_project.button import Button
from omok_project.omok5 import get_font


pygame.init()  # 파이 게임 초기화
screen = pygame.display.set_mode((600, 800))  # 화면 크기 설정
clock = pygame.time.Clock()


def main():

    while True:
        screen.fill((255, 255, 255))

        MENU_MOUSE_POS = pygame.mouse.get_pos()

        PLAY_BUTTON = Button(image=pygame.image.load("image/Play Rect.png"), pos=(200, 350),
                             text_input="Play", font=get_font(50), base_color="#d7fcd4", hovering_color="White")

        PLAY_BUTTON.changeColor(MENU_MOUSE_POS)
        PLAY_BUTTON.update(screen)

        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                pygame.exit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if PLAY_BUTTON.checkForInput(MENU_MOUSE_POS):
                    pan()

        pygame.display.update()


def pan():

    while True:
        screen.fill((0, 0, 0))

        pygame.display.update()


if __name__ == '__main__':
    main()
