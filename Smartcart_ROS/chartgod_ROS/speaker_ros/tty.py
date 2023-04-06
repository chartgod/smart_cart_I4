import os
from google.oauth2 import service_account
from google.cloud import texttospeech

# 구글 서비스 계정 키 파일 경로 설정
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/chart/catkin_ws/src/smartcart0510-6c5138c55e55.json'

# 클라이언트 초기화
client = texttospeech.TextToSpeechClient()

# 출력할 텍스트 입력
text = "아이포 로봇이 D구역으로 도착했습니다."

# 음성 파일 생성 요청 설정
synthesis_input = texttospeech.SynthesisInput(text=text)
voice = texttospeech.VoiceSelectionParams(language_code='ko-KR', ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL)
audio_config = texttospeech.AudioConfig(audio_encoding=texttospeech.AudioEncoding.MP3)

# 음성 파일 생성 요청 보내기
response = client.synthesize_speech(input=synthesis_input, voice=voice, audio_config=audio_config)

# 음성 파일 저장하기
with open('A1.mp3', 'wb') as out:
    out.write(response.audio_content)
    print('Audio content written to file "D1.mp3"')
