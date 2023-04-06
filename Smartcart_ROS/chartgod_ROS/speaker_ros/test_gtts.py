# -*- coding: utf-8 -*-
from gtts import gTTS
import os
import requests

certfile = "/path/to/certfile.pem"
text = u'아이포 로봇이 도착하였습니다.'

tts = gTTS(text=text, lang='ko')
with open("arrival.mp3", "wb") as f:
    tts.write_to_fp(f)
with open("arrival.mp3", "rb") as f:
    response = requests.post('https://translate.google.com/translate_tts', data=f.read(), headers={'content-type': 'audio/mp3'}, verify=False)
with open("arrival.mp3", "wb") as f:
    f.write(response.content)
os.system('mpg321 arrival.mp3')





'''이 코드에서는, 먼저 text 변수를 생성할 때, u 접두사를 붙이고 유니코드 문자열을 사용합니다. 그리고 gTTS 클래스의 save() 메서드에 encoding='utf-8' 매개변수를 추가하여 저장합니다. 마지막으로, 저장된 MP3 파일을 재생하는 명령어를 실행합니다.'''
