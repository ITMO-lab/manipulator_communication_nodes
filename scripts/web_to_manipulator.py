#!/usr/bin/env python
import requests
import json
import rospy
from std_msgs.msg import Float64MultiArray, String

WEB_SERVER = "https://desolate-taiga-99175.herokuapp.com/get_json"
REAGENTS_DESCRIPTION = ["ReagentNum", "C", "W", "p", "row", "column"]
MIXING_DESCRIPTION = ["ReagentNum1", "ReagentNum2", "C", "W", "p", "row", "column", "matrix"]


def web_to_manipulator():
    reagents_pub = rospy.Publisher('reagents', Float64MultiArray, queue_size=10)
    mixing_pub = rospy.Publisher('mixing', Float64MultiArray, queue_size=10)
    reagents_description_pub = rospy.Publisher('reagents_description', String, queue_size=10)
    mixing_description_pub = rospy.Publisher('mixing_description', String, queue_size=10)
    rospy.init_node('web_to_manipulator', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        reagents_description_pub.publish(str(REAGENTS_DESCRIPTION))
        mixing_description_pub.publish(str(MIXING_DESCRIPTION))

        try:
            response = requests.get(WEB_SERVER, timeout=10)
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
