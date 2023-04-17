'''작성자 이승헌'''
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>
#include <pocketsphinx.h>

std::string get_recognized_phrase(ps_decoder_t *ps, ad_rec_t *ad_rec) {
    int16 buf[2048];
    uint8 utt_started, in_speech;
    int32 k;
    std::string recognized_phrase;

    utt_started = FALSE;
    ps_start_utt(ps);

    while (1) {
        if ((k = ad_read(ad_rec, buf, 2048)) < 0)
            E_FATAL("Failed to read audio\n");
        ps_process_raw(ps, buf, k, FALSE, FALSE);
        in_speech = ps_get_in_speech(ps);
        if (in_speech && !utt_started) {
            utt_started = TRUE;
        }
        if (!in_speech && utt_started) {
            ps_end_utt(ps);
            recognized_phrase = ps_get_hyp(ps, NULL);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return recognized_phrase;
}

int main(int argc, char *argv[]) {
    cmd_ln_t *config;
    ps_decoder_t *ps;
    ad_rec_t *ad_rec;
    config = cmd_ln_init(NULL, ps_args(), TRUE,
                         "-hmm", "/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us",
                         "-lm", "/home/chart/.local/lib/python3.8/site-packages/pocketsphinx/model/en-us/en-us.lm.bin",
                         "-dict", "/home/chart/catkin_ws/src/my_dict.dict",
                         NULL);

    if (config == NULL) {
        std::cerr << "Failed to create config object, see log for details" << std::endl;
        return 1;
    }

    ps = ps_init(config);
    if (ps == NULL) {
        std::cerr << "Failed to create recognizer, see log for details" << std::endl;
        return 1;
    }

    if ((ad_rec = ad_open_sps(20000)) == NULL) {
        std::cerr << "Failed to open audio device" << std::endl;
        return 1;
    }

    std::cout << "음성 인식을 시작합니다. tracker를 말해주세요." << std::endl;

    while (1) {
        std::string recognized_phrase = get_recognized_phrase(ps, ad_rec);
        std::istringstream iss(recognized_phrase);
        std::string word;
        bool found_tracker = false;

        while (iss >> word) {
            if (word == "tracker") {
                found_tracker = true;
                break;
            }
        }

        if (found_tracker) {
            std::cout << "인식된 명령어: " << recognized_phrase << std::endl;
            // yolov5_person.py 실행 및 종료 코드를 여기에 추가하세요.
        } else {
            std::cout << "인식되지 않은 명령어: " << recognized_phrase << std::endl;
        }
    }

    ad_close(ad_rec);
    ps_free(ps);
    cmd_ln_free_r(config);

    return 0;
}
