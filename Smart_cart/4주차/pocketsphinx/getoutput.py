import os
from pocketsphinx import LiveSpeech, get_model_path

def get_sphinx_output():
    model_path = get_model_path()
    speech = LiveSpeech(
        verbose=False,
        sampling_rate=16000,
        buffer_size=2048,
        no_search=False,
        full_utt=False,
        hmm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us'),
        lm=os.path.join(model_path, '/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us.lm.bin'),
        dic=os.path.join(model_path, '/home/chart/catkin_ws/src/my_dict.dict')
    )

    for phrase in speech:
        sphinx_output = str(phrase)
        return sphinx_output
