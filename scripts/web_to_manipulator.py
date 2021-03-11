#!/usr/bin/env python
import requests
import json
import rospy
from std_msgs.msg import Float64MultiArray, String, Int32MultiArray

ROSTOPIC_PREFIX = ""
WEB_SERVER = "https://desolate-taiga-99175.herokuapp.com/get_json"
REAGENTS_DESCRIPTION = ["ReagentNum", "C", "W", "p", "row", "column"]
MIXING_DESCRIPTION = ["ReagentNum1", "ReagentNum2", "C", "W", "p", "row", "column", "matrix"]
global MANIPULATORS_STATES
MANIPULATORS_STATES = []


def set_manipulators_states(data):
    global MANIPULATORS_STATES
    MANIPULATORS_STATES = data.data


def get_manipulators_states():
    global MANIPULATORS_STATES
    if len(MANIPULATORS_STATES) == 0:
        return ""
    return "?states=" + ','.join(map(str, MANIPULATORS_STATES))


def web_to_manipulator():
    global MANIPULATORS_STATES
    manipulators_status_sub = rospy.Subscriber(ROSTOPIC_PREFIX + 'manipulators_status', Int32MultiArray,
                                               set_manipulators_states)
    reagents_pub = rospy.Publisher(ROSTOPIC_PREFIX + 'reagents', Float64MultiArray, queue_size=10)
    mixing_pub = rospy.Publisher(ROSTOPIC_PREFIX + 'mixing', Float64MultiArray, queue_size=10)
    reagents_description_pub = rospy.Publisher(ROSTOPIC_PREFIX + 'reagents_description', String, queue_size=10)
    mixing_description_pub = rospy.Publisher(ROSTOPIC_PREFIX + 'mixing_description', String, queue_size=10)
    rospy.init_node(ROSTOPIC_PREFIX + 'web_to_manipulator', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        reagents_description_pub.publish(str(REAGENTS_DESCRIPTION))
        mixing_description_pub.publish(str(MIXING_DESCRIPTION))

        try:
            response = requests.get(WEB_SERVER + get_manipulators_states(), timeout=10)
        except KeyboardInterrupt:
            break
        except requests.Timeout:
            continue
        except requests.ConnectionError:
            rospy.logerr("Web server is unreachable")
            continue
        try:
            parsed_data = json.loads(response.text)
        except KeyboardInterrupt:
            break
        except:
            rospy.logerr("Web server is sending incorrect or broken json")
            continue
        rospy.logdebug(str(parsed_data))
        if len(parsed_data) == 0:
            continue

        reagents = Float64MultiArray()
        try:
            for reagent in parsed_data.get("reagents"):
                for key in reagent.keys():
                    reagent[key] = str(
                        reagent[key])  # Crutch to handle incoming values as strings and numbers interspersed.
                reagents.data = [float(reagent.get(key) or 'nan') for key in REAGENTS_DESCRIPTION]
                reagents_pub.publish(reagents)
        except KeyboardInterrupt:
            break
        except:
            rospy.logerr(
                "Web server is sending incorrect fields contains for reagents\nThis node needs \"\" or any real number as a string")
            continue

        try:
            for reagent in parsed_data.get("mixing"):
                for key in reagent.keys():
                    reagent[key] = str(
                        reagent[key])  # Crutch to handle incoming values as strings and numbers interspersed.
                reagents.data = [float(reagent.get(key) or 'nan') for key in MIXING_DESCRIPTION]
                mixing_pub.publish(reagents)
        except KeyboardInterrupt:
            break
        except:
            rospy.logerr(
                "Web server is sending incorrect fields contains for mixing\nThis node needs \"\" or any real number as a string")
            continue

        rate.sleep()


if __name__ == '__main__':
    try:
        web_to_manipulator()
    except rospy.ROSInterruptException:
        pass
