# 실행 소스 코드 

먼저 다운로드를 해줘야 한다.
``` bash
!curl -L "https://app.roboflow.com/ds/DKPK8xSzt4?key=M7X5z38Mzu" > roboflow.zip; unzip roboflow.zip; rm roboflow.zip
```

그 다음 yolov5를 다운해준다.

``` bash
%cd /content/
!git clone https://github.com/ultralytics/yolov5.git
```

requirements.txt 설치

``` bash
%cd /content/yolov5
!pip install -r requirements.txt
```

glob 활용

``` bash
%cd /
from glob import glob
```

list 만들기
``` bash
img_list = glob('/content/dataset/test/images/*.jpg')
img_list = glob('/content/dataset/val/images/*.jpg')
img_list = glob('/content/dataset/train/images/*.jpg')
print(len(img_list))
```


``` bash
train_val_img_list, test_img_list = train_test_split(img_list, test_size=0.2, random_state=2000)
train_img_list, val_img_list = train_test_split(train_val_img_list, test_size=0.25, random_state=2000)
```
출력
``` bash
print(len(train_img_list), len(val_img_list), len(test_img_list))

```
---------------------------------------------------------------------------


``` bash

with open('/content/dataset/train.txt', 'w') as f:
  f.write('\n'.join(train_img_list) + '\n')

with open('/content/dataset/val.txt', 'w') as f:
  f.write('\n'.join(val_img_list) + '\n')

with open('/content/dataset/test.txt', 'w') as f:
  f.write('\n'.join(test_img_list) + '\n')
  
  import yaml

with open('/content/dataset/data.yaml', 'r') as f:
    data = yaml.load(f, Loader=yaml.FullLoader)

print(data)

data['train'] = '/content/dataset/train.txt'
data['val'] = '/content/dataset/val.txt'
data['test'] = '/content/dataset/test.txt'
with open('/content/dataset/data.yaml', 'w') as f:
  yaml.dump(data, f)

print(data)
```
실행 하기 전에 **경로설정** 중요합니다.

``` bash

%cd /content/yolov5/
!python train.py --img 416 --batch 16 --epochs 50 --data /content/dataset/data.yaml --cfg ./models/yolov5s.yaml --weights yolov5s.pt --name gun_yolov5s_results

```

