from utils import *


class DataSerializer:
    """
    Provides utility methods to convert analyzed data
    into structured formats for server transmission.
    """

    @staticmethod
    def to_json(data: dict) -> str:
        """
        Converts a Python dictionary containing analyzed data
        into a formatted JSON string suitable for sending to the server.

        :param data: The dictionary containing the analyzed data.
        :type data: dict
        :return: A JSON-formatted string representation of `data`.
        :rtype: str
        """

        return json.dumps(data, ensure_ascii=False, indent=2)

    @staticmethod
    def to_dict(DE) -> dict:
        """
        Converts a data entity object containing analyzed results into a structured dictionary.

        :param DE: An instance of `DataExtractor` class with following attributes:
                   - `id` (int): Six-digit user ID
                   - `timestamps` (list of float): Timestamps of [videoStart, shotTime, videoEnd]
                   - `mode` (str): Specified mode received from Arduino
                   - `trajectory` (list of [x, y]): Ball trajectory coordinates
                   - `angles` (list of float): [elbow, wrist] angles in degrees
                   - `height` (int): Player height in pixels within the frame
                   - `h_loc` (float): Ball horizontal position within shoulder width
                   - `v_loc` (float): Ball vertical position normalized by player height
        :type DE: DataExtractor
        :return: A structured dictionary representing the analyzed data.
        :rtype: dict
        """

        data = {
            "userID": DE.id,

            "timestamp": {
                "videoStart": DE.timestamps[0],
                "shotTime": DE.timestamps[1],
                "videoEnd": DE.timestamps[2]
            },

            "mode": DE.mode,

            "ballTrajectory": DE.trajectory,

            "jointAngles": {
                "elbow": DE.angles[0],
                "wrist": DE.angles[1]
            },

            "ballReleaseMoment": {
                "height": DE.height,
                "ballLocation": {
                    "horizontal": DE.h_loc,
                    "vertical": DE.v_loc
                }
            }
        }

        return data
