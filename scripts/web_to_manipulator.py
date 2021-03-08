#!/usr/bin/env python
import requests
import json
import rospy
from std_msgs.msg import Float64MultiArray, Float64, String

WEB_SERVER = "https://desolate-taiga-99175.herokuapp.com/get_json"
REAGENTS_DESCRIPTION = ["ReagentNum", "C", "W", "p", "row", "column"]
MIXING_DESCRIPTION = ["ReagentNum1", "ReagentNum2", "C", "W", "p", "row", "column", "matrix"]

TEST_WEB_RESPONSE = """
{
  "reagents": [
    {
      "ReagentName": "Cl",
      "ReagentNum": "0",
      "C": "1",
      "W": "0.035453000000000005",
      "p": "1",
      "row": "0",
      "column": "0"
    },
    {
      "ReagentName": "C2H6O",
      "ReagentNum": "1",
      "C": "1",
      "W": "0.035453000000000005",
      "p": "1",
      "row": "0",
      "column": "1"
    },
    {
      "ReagentName": "H2O",
      "ReagentNum": "-1",
      "C": "1",
      "W": "1",
      "p": "1",
      "row": "0",
      "column": "2"
    },
    {
      "ReagentName": "H2O",
      "ReagentNum": "-1",
      "C": "1",
      "W": "1",
      "p": "1",
      "row": "1",
      "column": "0"
    }
  ],
  "mixing": [
    {
      "ReagentName1": "C2H6O",
      "ReagentNum1": "1",
      "ReagentName2": "H2O",
      "ReagentNum2": "-1",
      "C": "1",
      "W": "0.035453000000000005",
      "p": "1",
      "row": "0",
      "column": "0"
    },
    {
      "ReagentName1": "C2H6O",
      "ReagentNum1": "1",
      "ReagentName2": "H2O",
      "ReagentNum2": "-1",
      "C": "1",
      "W": "",
      "p": "",
      "row": "",
      "column": "0"
    }
  ]
}
"""

TEST_WEB_RESPONCE_WRONG = """
{u'mixing': [{u'C': u'321', u'Reagent1Num': 2, u'column': 1, u'Reagent2Num': -1, u'p': u'321', u'ReagentName1': u'Cl', u'W': u'123', u'ReagentName2': u'H2O', u'row': 1, u'matrix': 1}, {u'C': u'123', u'Reagent1Num': 1, u'column': 1, u'Reagent2Num': -1, u'p': u'321', u'ReagentName1': u'Li', u'W': u'321', u'ReagentName2': u'H2O', u'row': 2, u'matrix': 1}], u'reagents': [{u'C': u'12', u'ReagentName': u'H20', u'column': 0, u'p': u'213', u'ReagentNum': 0, u'W': u'32', u'row': 0}, {u'C': u'1952.494', u'ReagentName': u'Li', u'column': 1, u'p': u'123', u'ReagentNum': 1, u'W': u'11.0181', u'row': 0}, {u'C': u'', u'ReagentName': u'Li', u'column': 2, u'p': u'1', u'ReagentNum': 1, u'W': u'', u'row': 0}, {u'C': u'', u'ReagentName': u'Cl', u'column': 0, u'p': u'1', u'ReagentNum': 2, u'W': u'', u'row': 1}, {u'C': u'', u'ReagentName': u'Cl', u'column': 1, u'p': u'1', u'ReagentNum': 2, u'W': u'', u'row': 2}, {u'C': u'', u'ReagentName': u'Li', u'column': 2, u'p': u'1', u'ReagentNum': 1, u'W': u'', u'row': 2}]}
"""


def talker():
    reagents_pub = rospy.Publisher('reagents', Float64MultiArray, queue_size=10)
    mixing_pub = rospy.Publisher('mixing', Float64MultiArray, queue_size=10)
    reagents_description_pub = rospy.Publisher('reagents_description', String, queue_size=10)
    mixing_description_pub = rospy.Publisher('mixing_description', String, queue_size=10)
    rospy.init_node('web_to_manipulator', anonymous=True)
    while not rospy.is_shutdown():
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

        reagents_description_pub.publish(str(REAGENTS_DESCRIPTION))
        mixing_description_pub.publish(str(MIXING_DESCRIPTION))


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
