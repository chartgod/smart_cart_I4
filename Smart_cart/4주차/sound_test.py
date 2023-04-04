import sounddevice as sd
from pocketsphinx import LiveSpeech

def get_sphinx_output():
    # 사용 가능한 장치 확인
    print(sd.query_devices())

    # 사용할 입력 장치와 출력 장치 지정
    input_device = sd.default.device[0]
    print(input_device)
    output_device = sd.default.device[1]
    print(output_device)

    # LiveSpeech 인스턴스 생성
    speech = LiveSpeech(
        verbose=False,
        sampling_rate=16000,
        buffer_size=2048,
        no_search=False,
        full_utt=False,
        lm=False,
        dic=False,
        input_device=input_device,
        output_device=output_device
    )

    # 인식 결과 반환
    for phrase in speech:
        return phrase