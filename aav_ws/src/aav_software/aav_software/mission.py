from enum import Enum


class Mission(Enum):
    """
    Current mission being run by the Object Alignment Controller.
    Used to determine which parameters to use and which actions to take at various stages of the state machine.
    """

    PACKAGE_DELIVERY_CUASC = 0  # Mission for delivering cube onto the bullseye. Drone will land and take off autonomously.
    PAYLOAD_DROP_CUASC = 1  # Mission for dropping beanbag onto the bullseye. Drone remains in air the entire time.
    GCP_MARKER_ALIGNING_CUASC = (
        2  # Mission for drone only aligning to GCP points and not doing anything else.
    )
    PAYLOAD_DELIVERY_SUAS = 3  # Mission for delivering water bottle/strobe beacon to detected object on the ground. Drone stays in the air the entire time.
