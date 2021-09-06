#!/usr/bin/env python3
# Standard, System and Third Party
from os import path

# ROS
# import rospkg
import rospy
from std_msgs.msg import String


# Robonomics
from robonomics_msgs.msg import Demand
from ethereum_common.msg import Address, UInt256
from ethereum_common.srv import BlockNumber
from ipfs_common.msg import Multihash


def make_deadline():
    lifetime = 1000
    deadline = rospy.ServiceProxy('/eth/current_block', BlockNumber)().number + lifetime
    return str(deadline)


def create_demand(fields: dict) -> Demand:
    rospy.loginfo("Creating a demand...")

    demand = Demand()
    demand.model = Multihash(fields["model"]["multihash"])
    demand.objective = Multihash(fields["objective"]["multihash"])
    demand.token = Address(fields["token"]["address"])
    demand.cost = UInt256(fields["cost"]["uint256"])
    demand.lighthouse = Address(fields["lighthouse"]["address"])
    demand.validator = Address(fields["validator"]["address"])
    demand.validatorFee = UInt256(fields["validatorFee"]["uint256"])
    demand.deadline = UInt256(fields["deadline"]["uint256"])

    rospy.loginfo(demand)
    return demand


def send_demand():
    # Build demand
    fields = {
        "model": {
            "multihash": "QmZq9axSqwSVdGsJ4HQ5Mjucu1CW2xAWecmXCb5Y5ku47T"
        },
        "objective": {
            "multihash": "QmTGY5N4XSqYivgN1aVxtwx27s5ChC4KoJekq7gjShuGyA"
        },
        "token": {
            "address": "0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2"
        },
        "cost": {
            "uint256": "7500000000000000"
        },
        "validator": {
            "address": "0x0000000000000000000000000000000000000000"
        },
        "lighthouse": {
            "address": "0xD40AC7F1e5401e03D00F5aeC1779D8e5Af4CF9f1"
        },
        "validatorFee": {
            "uint256": "0"
        },
        "deadline": {
            "uint256": make_deadline()
        }
    }

    demand = create_demand(fields)  # put the fields into Demand structure

    rospy.sleep(2)  # it's necessary to make sure ROS has registered the publisher
    demand_pub.publish(demand)
    rospy.loginfo("Published!")


def check_consumables_remains(data: str):
    """
    Check the amount of canvases left using a txt-file with a number as a tracker
    if less than 2, send demand
    """
    if data.data != "stop":
        return False

    try:
        f = open("/home/kuka/consumables_orderer/scripts/consumables_remains.txt", "r+")
        number = int(f.read())
        f.seek(0)
        f.truncate()
        rospy.loginfo(f"Previous number of canvases: {number}")
    except Exception as e:
        rospy.loginfo("can't open the file!")
        rospy.loginfo(e)
        exit()
    number -= 1
    rospy.loginfo(f"Current number of canvases: {number}")
    if number == 1:
        rospy.loginfo(f"Need to order canvases.")
        f.write("5")
        f.close()
        try:
            send_demand()
        except Exception as e:
            rospy.logerr(f"Error in sending demand {e}")
    else:
        f.write(str(number))
        f.close()
        rospy.loginfo(f"Can continue drawing.")


if __name__ == "__main__":
    rospy.init_node("consumables_orderer")
    rospy.loginfo("Node \"consumables_orderer\" initiated")

    # rospack = rospkg.RosPack()
    # packagePath = rospack.get_path("consumables_orderer") + "/"

    # Register drawing process topic listener (/film with a start|stop messages)
    rospy.Subscriber("film", String, check_consumables_remains)
    rospy.loginfo("Listening to /film")
    # Register a publisher
    demand_pub = rospy.Publisher("/liability/infochan/eth/signing/demand", Demand, queue_size=128)
    rospy.spin()  # press Ctrl+C to exit
