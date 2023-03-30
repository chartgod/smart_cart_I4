import os
from pocketsphinx import LiveSpeech, get_model_path

model_path = get_model_path()

# 사용자가 입력한 값으로 설정
sampling_rate = int(input("마이크 또는 오디오 파일의 샘플링 레이트를 입력하세요: "))
buffer_size = int(input("마이크 또는 오디오 파일의 버퍼 사이즈를 입력하세요: "))

# 새로운 사전 파일을 사용하여 LiveSpeech 객체를 생성
speech = LiveSpeech(
    verbose=False,
    sampling_rate=sampling_rate,
    buffer_size=buffer_size,
    no_search=False,
    full_utt=False,
    hmm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us'),
    lm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us.lm.bin'),
    dic=os.path.join(os.path.dirname(os.path.realpath(__file__)), "/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/my_dict.dict"))

dict_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/my_dict.dict")
print("사용 중인 사전 파일 경로:", dict_path)

for phrase in speech:
    # 반환된 값을 문자열로 변환한 후에 lower() 메서드를 사용
    result = "음성인식 결과: {}".format(str(phrase).lower().strip())
    print(result)
