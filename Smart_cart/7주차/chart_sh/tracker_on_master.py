# -*- coding: utf-8 -*-
# 작성자: 이승헌 _연희직업전문학교
# 0415
import os
import subprocess
from pocketsphinx import LiveSpeech


def get_sphinx_output(speech):
    for phrase in speech:
        recognized_phrase = str(phrase).lower().strip()
        if recognized_phrase in ['tracker']:
            print("인식된 명령어:", recognized_phrase)
            return recognized_phrase
        else:
            print("인식되지 않은 명령어:", recognized_phrase)


def get_model_path():
    return os.path.join(os.path.expanduser("~"), ".local/lib/python3.8/site-packages/pocketsphinx/model/en-us")


def run_tracker():
    # test1.py 실행 명령
    command = "python3 yolov5_person.py"

    model_path = get_model_path()
    speech = LiveSpeech(
        verbose=False,
        sampling_rate=20000,  # 16000, 2048 use O
        buffer_size=2048,
        no_search=False,
        full_utt=False,
        hmm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us'),
        lm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us.lm.bin'),
        dic=os.path.join(model_path,'/home/chart/catkin_ws/src/my_dict.dict')
    )

    # 음성 인식을 시작합니다.
    print("음성 인식을 시작합니다. tracker를 말해주세요.")

    yolov5_person_process = None

    # 음성 인식이 계속되는 동안 반복
    while True:
        recognized_phrase = get_sphinx_output(speech)
        if recognized_phrase == "tracker":
            if yolov5_person_process is None or yolov5_person_process.poll() is not None:
                # test1.py 실행
                yolov5_person_process = subprocess.Popen(command.split())
            else:
                # test1.py 종료
                yolov5_person_process.terminate()
                yolov5_person_process.wait()
                yolov5_person_process = None


if __name__ == "__main__":
    run_tracker()
