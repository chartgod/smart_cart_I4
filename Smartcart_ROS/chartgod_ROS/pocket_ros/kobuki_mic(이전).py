import os
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pocketsphinx import LiveSpeech, get_model_path

rospy.init_node('pocket_sphinx_controller', anonymous=True)
kobuki_velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def process_sphinx_result(sphinx_output):
    kobuki_twist = Twist()

    if sphinx_output == "a":
        kobuki_twist.linear.x = 0.5
        kobuki_twist.angular.z = 0.0
    elif sphinx_output == "b":
        kobuki_twist.linear.x = -0.5
        kobuki_twist.angular.z = 0.0
    elif sphinx_output == "c":
        kobuki_twist.linear.x = 0.0
        kobuki_twist.angular.z = 1.0
    else:
        kobuki_twist.linear.x = 0.0
        kobuki_twist.angular.z = 0.0

    return kobuki_twist

def get_sphinx_output():
    model_path = get_model_path()
    speech = LiveSpeech(
        verbose=False,
        sampling_rate=16000,
        buffer_size=2048,
        no_search=False,
        full_utt=False,
        hmm=os.path.join(model_path, 'en-us'),
        lm=os.path.join(model_path, 'en-us.lm.bin'),
        dic=os.path.join(model_path, 'cmudict-en-us.dict')
    )

    for phrase in speech:
        sphinx_output = str(phrase)
        return sphinx_output

if __name__ == '__main__':
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        sphinx_output = get_sphinx_output()
        kobuki_twist = process_sphinx_result(sphinx_output)
        kobuki_velocity_pub.publish(kobuki_twist)
        rate.sleep()
