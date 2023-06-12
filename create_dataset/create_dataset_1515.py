import numpy as np
from glob import glob
from tqdm import tqdm  # for 문의 상태바를 나태내주는 라이브러리
import os

# 데이터 전처리 과정
game_rule = 'Standard'  # 프리스타일(3x3가능 5개 이상만 돌을 연속으로 두면 승리)
base_path = 'C:/study/Omok_robot/gomocup2021results'  # 전처리전 데이터가 저장되어 있는 경로

output_path = os.path.join('dataset_1515', os.path.basename(base_path))
os.makedirs(output_path, exist_ok=True)  # 디렉토리 생성

# 경로에 있는 모든 psq파일을 file_list에 저장
file_list = glob(os.path.join(base_path, '%s*/*.psq' % (game_rule, )))

for index, file_path in enumerate(tqdm(file_list)):
    with open(file_path, 'r') as f:
        lines = f.read().splitlines()

    w, h = lines[0].split(' ')[1].strip(',').split('x')  # 가로 X 세로의 크기 추출
    w, h = int(w), int(h)  # 문자에서 인트형으로 형 변환

    lines = lines[1:]  # 첫번째 줄을 제외한 나머지 줄

    inputs, outputs = [], []
    board = np.zeros([h, w], dtype=np.int8)

    for i, line in enumerate(lines):
        if ',' not in line:
            break
        x, y, t = np.array(line.split(','), np.int8)

        if i % 2 == 0:  # 오목의 순서제
            player = 1
        else:
            player = 2

        input1 = board.copy().astype(np.int8)
        input1[(input1 != player) & (input1 != 0)] = -1
        input1[(input1 == player) & (input1 != 0)] = 1

        output = np.zeros([h, w], dtype=np.int8)
        output[y-1, x-1] = 1

        # h w 순서

        # augmentation
        # rotate 4 x flip 3 = 12
        # 데이터셋을 늘림

        for k in range(4):
            input_rot = np.rot90(input1, k=k)
            output_rot = np.rot90(output, k=k)

            inputs.append(input_rot)
            outputs.append(output_rot)

            inputs.append(np.fliplr(input_rot))
            outputs.append(np.fliplr(output_rot))

            inputs.append(np.flipud(input_rot))
            outputs.append(np.flipud(output_rot))

        # update board
        board[y-1, x-1] = player
        # print(board)

# dataset 저장
    np.savez_compressed(os.path.join(output_path, '%s.npz' % (str(index).zfill(5))), inputs=inputs, outputs=outputs)
